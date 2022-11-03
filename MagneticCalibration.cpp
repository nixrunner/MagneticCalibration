#include <AP_Compass.h>
#include <MagneticCalibration.h>
#include <AP_HAL.h>
#include <AP_GeodesicGrid.h>
#include <AP_AHRS.h>
#include <AP_GPS.h>
#include <GCS.h>
#include <AP_InternalError.h>


#define FIELD_RADIUS_MIN 150
#define FIELD_RADIUS_MAX 950

////////////////PUBLIC INTERFACE HERE//////////////
///////////////////////////////////////////////////
//////////////////////////////////////////////////


CompassCalibrator::CompassCalibrator()
{
    set_status(Status::NOT_STARTED);
}

// Request to cancel calibration 
void CompassCalibrator::stop()
{
    _requested_status = Status::NOT_STARTED;
    _status_set_requested = true;
}

void CompassCalibrator::set_orientation(enum Rotation orientation, bool is_external, bool fix_orientation, bool always_45_deg)
{
    cal_settings.check_orientation = true;
    cal_settings.orientation = orientation;
    cal_settings.orig_orientation = orientation;
    cal_settings.is_external = is_external;
    cal_settings.fix_orientation = fix_orientation;
    cal_settings.always_45_deg = always_45_deg;
}

void CompassCalibrator::start(bool retry, float delay, uint16_t offset_max, uint8_t compass_idx, float tolerance)
{
    if (compass_idx > COMPASS_MAX_INSTANCES) {
        return;
    }

    // Don't do this while we are already started
    if (_running()) {
        return;
    }
    cal_settings.offset_max = offset_max;
    cal_settings.attempt = 1;
    cal_settings.retry = retry;
    cal_settings.delay_start_sec = delay;
    cal_settings.start_time_ms = AP_HAL::millis();
    cal_settings.compass_idx = compass_idx;
    cal_settings.tolerance = tolerance;

    // Request status change to Waiting to start
    _requested_status = Status::WAITING_TO_START;
    _status_set_requested = true;
}

// Record point mag sample and associated attitude sample to intermediate struct
void CompassCalibrator::new_sample(const Vector3f& sample)
{
    WITH_SEMAPHORE(sample_sem);
    _last_sample.set(sample);
    _last_sample.att.set_from_ahrs();
    _new_sample = true;
}

bool CompassCalibrator::failed() {
    return (cal_state.status == Status::FAILED ||
            cal_state.status == Status::BAD_ORIENTATION || 
            cal_state.status == Status::BAD_RADIUS);
}


bool CompassCalibrator::running() { 
    return (cal_state.status == Status::RUNNING_STEP_ONE || cal_state.status == Status::RUNNING_STEP_TWO);
}

const CompassCalibrator::Report CompassCalibrator::get_report() {
    return cal_report;
}

const CompassCalibrator::State CompassCalibrator::get_state() {
    return cal_state;
}

//////////////PRIVATE INTERFACE HERE/////////
///////////////////////////////////////////////
/////////////////////////////////////////////////
///////////////////////////////////////////////


void CompassCalibrator::new_sample(const Vector3f& sample)
{
    _last_sample.set(sample);
    _last_sample.att.set_from_ahrs();
    _new_sample = true;
}

void CompassCalibrator::update()
{

    //pickup samples from intermediate struct
    pull_sample();

    {

        //update_settings
        if (!running()) {
            update_cal_settings();
        }

        //update requested state
        if (_status_set_requested) {
            _status_set_requested = false;
            set_status(_requested_status);
        }
        //update report and status
        update_cal_status();
        update_cal_report();
    }

    // collect the minimum number of samples
    if (!_fitting()) {
        return;
    }

    if (_status == Status::RUNNING_STEP_ONE) {
        if (_fit_step >= 10) {
            if (is_equal(_fitness, _initial_fitness) || isnan(_fitness)) {  // if true, means that fitness is diverging instead of converging
                set_status(Status::FAILED);
            } else {
                set_status(Status::RUNNING_STEP_TWO);
            }
        } else {
            if (_fit_step == 0) {
                calc_initial_offset();
            }
            run_sphere_fit();
            _fit_step++;
        }
    } else if (_status == Status::RUNNING_STEP_TWO) {
        if (_fit_step >= 35) {
            if (fit_acceptable() && fix_radius() && calculate_orientation()) {
                set_status(Status::SUCCESS);
            } else {
                set_status(Status::FAILED);
            }
        } else if (_fit_step < 15) {
            run_sphere_fit();
            _fit_step++;
        } else {
            run_ellipsoid_fit();
            _fit_step++;
        }
    }
}

void CompassCalibrator::pull_sample()
{
    CompassSample mag_sample;
    {
        if (!_new_sample) {
            return;
        }
        if (_status == Status::WAITING_TO_START) {
            set_status(Status::RUNNING_STEP_ONE);
        }
        _new_sample = false;
        mag_sample = _last_sample;
    }
    if (_running() && _samples_collected < COMPASS_CAL_NUM_SAMPLES && accept_sample(mag_sample.get())) {
        update_completion_mask(mag_sample.get());
        _sample_buffer[_samples_collected] = mag_sample;
        _samples_collected++;
    }
}
// initialize fitness before starting a fit
void CompassCalibrator::initialize_fit()
{
    if (_samples_collected != 0) {
        _fitness = calc_mean_squared_residuals(_params);
    }
    else {
        _fitness = 1.0e30f;
    }
    _initial_fitness = _fitness;
    _sphere_lambda = 1.0f;
    _ellipsoid_lambda = 1.0f;
    _fit_step = 0;
}

void CompassCalibrator::reset_state()
{
    _samples_collected = 0;
    _samples_thinned = 0;
    _params.radius = 200;
    _params.offset.zero();
    _params.diag = Vector3f(1.0f, 1.0f, 1.0f);
    _params.offdiag.zero();
    _params.scale_factor = 0;

    memset(_completion_mask, 0, sizeof(_completion_mask));
    initialize_fit();
}

bool CompassCalibrator::set_status(CompassCalibrator::Status status)
{
    if (status != Status::NOT_STARTED && _status == status) {
        return true;
    }

    switch (status) {
    case Status::NOT_STARTED:
        reset_state();
        _status = Status::NOT_STARTED;
        if (_sample_buffer != nullptr) {
            free(_sample_buffer);
            _sample_buffer = nullptr;
        }
        return true;

    case Status::WAITING_TO_START:
        reset_state();
        _status = Status::WAITING_TO_START;
        set_status(Status::RUNNING_STEP_ONE);
        return true;

    case Status::RUNNING_STEP_ONE:
        if (_status != Status::WAITING_TO_START) {
            return false;
        }

        // on first attempt delay start if requested by caller
        if (_attempt == 1 && (AP_HAL::millis() - _start_time_ms) * 1.0e-3f < _delay_start_sec) {
            return false;
        }

        if (_sample_buffer == nullptr) {
            _sample_buffer = (CompassSample*)calloc(COMPASS_CAL_NUM_SAMPLES, sizeof(CompassSample));
        }
        if (_sample_buffer != nullptr) {
            initialize_fit();
            _status = Status::RUNNING_STEP_ONE;
            return true;
        }
        return false;

    case Status::RUNNING_STEP_TWO:
        if (_status != Status::RUNNING_STEP_ONE) {
            return false;
        }
        thin_samples();
        initialize_fit();
        _status = Status::RUNNING_STEP_TWO;
        return true;

    case Status::SUCCESS:
        if (_status != Status::RUNNING_STEP_TWO) {
            return false;
        }

        if (_sample_buffer != nullptr) {
            free(_sample_buffer);
            _sample_buffer = nullptr;
        }

        _status = Status::SUCCESS;
        return true;

    case Status::FAILED:
        if (_status == Status::BAD_ORIENTATION ||
            _status == Status::BAD_RADIUS) {
            // don't overwrite bad orientation status
            return false;
        }
        FALLTHROUGH;

    case Status::BAD_ORIENTATION:
    case Status::BAD_RADIUS:
        if (_status == Status::NOT_STARTED) {
            return false;
        }

        if (_retry && set_status(Status::WAITING_TO_START)) {
            _attempt++;
            return true;
        }

        if (_sample_buffer != nullptr) {
            free(_sample_buffer);
            _sample_buffer = nullptr;
        }

        _status = status;
        return true;

    default:
        return false;
    };
}

bool CompassCalibrator::fit_acceptable() const
{
    if (!isnan(_fitness) &&
        _params.radius > FIELD_RADIUS_MIN && _params.radius < FIELD_RADIUS_MAX &&
        fabsf(_params.offset.x) < _offset_max &&
        fabsf(_params.offset.y) < _offset_max &&
        fabsf(_params.offset.z) < _offset_max &&
        _params.diag.x > 0.2f && _params.diag.x < 5.0f &&
        _params.diag.y > 0.2f && _params.diag.y < 5.0f &&
        _params.diag.z > 0.2f && _params.diag.z < 5.0f &&
        fabsf(_params.offdiag.x) < 1.0f &&      //absolute of sine/cosine output cannot be greater than 1
        fabsf(_params.offdiag.y) < 1.0f &&
        fabsf(_params.offdiag.z) < 1.0f) {
        return _fitness <= sq(_tolerance);
    }
    return false;
}

void CompassCalibrator::thin_samples()
{
    if (_sample_buffer == nullptr) {
        return;
    }

    _samples_thinned = 0;
    // shuffle the samples http://en.wikipedia.org/wiki/Fisher%E2%80%93Yates_shuffle
    // this is so that adjacent samples don't get sequentially eliminated
    for (uint16_t i = _samples_collected - 1; i >= 1; i--) {
        uint16_t j = get_random16() % (i + 1);
        CompassSample temp = _sample_buffer[i];
        _sample_buffer[i] = _sample_buffer[j];
        _sample_buffer[j] = temp;
    }

    // remove any samples that are close together
    for (uint16_t i = 0; i < _samples_collected; i++) {
        if (!accept_sample(_sample_buffer[i], i)) {
            _sample_buffer[i] = _sample_buffer[_samples_collected - 1];
            _samples_collected--;
            _samples_thinned++;
        }
    }

    update_completion_mask();
}

/*
 * The sample acceptance distance is determined as follows:
 * For any regular polyhedron with triangular faces, the angle theta subtended
 * by two closest points is defined as
 *
 *      theta = arccos(cos(A)/(1-cos(A)))
 *
 * Where:
 *      A = (4pi/F + pi)/3
 * and
 *      F = 2V - 4 is the number of faces for the polyhedron in consideration,
 *      which depends on the number of vertices V
 *
 * The above equation was proved after solving for spherical triangular excess
 * and related equations.
 */
bool CompassCalibrator::accept_sample(const Vector3f& sample, uint16_t skip_index)
{
    static const uint16_t faces = (2 * COMPASS_CAL_NUM_SAMPLES - 4);
    static const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    static const float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));

    if (_sample_buffer == nullptr) {
        return false;
    }

    float min_distance = _params.radius * 2 * sinf(theta / 2);

    for (uint16_t i = 0; i < _samples_collected; i++) {
        if (i != skip_index) {
            float distance = (sample - _sample_buffer[i].get()).length();
            if (distance < min_distance) {
                return false;
            }
        }
    }
    return true;
}

bool CompassCalibrator::accept_sample(const CompassSample& sample, uint16_t skip_index)
{
    return accept_sample(sample.get(), skip_index);
}

float CompassCalibrator::calc_residual(const Vector3f& sample, const param_t& params) const
{
    Matrix3f softiron(
        params.diag.x, params.offdiag.x, params.offdiag.y,
        params.offdiag.x, params.diag.y, params.offdiag.z,
        params.offdiag.y, params.offdiag.z, params.diag.z
    );
    return params.radius - (softiron * (sample + params.offset)).length();
}

// calc the fitness given a set of parameters (offsets, diagonals, off diagonals)
float CompassCalibrator::calc_mean_squared_residuals(const param_t& params) const
{
    if (_sample_buffer == nullptr || _samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for (uint16_t i = 0; i < _samples_collected; i++) {
        Vector3f sample = _sample_buffer[i].get();
        float resid = calc_residual(sample, params);
        sum += sq(resid);
    }
    sum /= _samples_collected;
    return sum;
}

// calculate initial offsets by simply taking the average values of the samples
void CompassCalibrator::calc_initial_offset()
{
    // Set initial offset to the average value of the samples
    _params.offset.zero();
    for (uint16_t k = 0; k < _samples_collected; k++) {
        _params.offset -= _sample_buffer[k].get();
    }
    _params.offset /= _samples_collected;
}

void CompassCalibrator::calc_sphere_jacob(const Vector3f& sample, const param_t& params, float* ret) const
{
    const Vector3f& offset = params.offset;
    const Vector3f& diag = params.diag;
    const Vector3f& offdiag = params.offdiag;
    Matrix3f softiron(
        diag.x, offdiag.x, offdiag.y,
        offdiag.x, diag.y, offdiag.z,
        offdiag.y, offdiag.z, diag.z
    );

    float A = (diag.x * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B = (offdiag.x * (sample.x + offset.x)) + (diag.y * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C = (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z * (sample.z + offset.z));
    float length = (softiron * (sample + offset)).length();

    // 0: partial derivative (radius wrt fitness fn) fn operated on sample
    ret[0] = 1.0f;
    // 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
    ret[1] = -1.0f * (((diag.x * A) + (offdiag.x * B) + (offdiag.y * C)) / length);
    ret[2] = -1.0f * (((offdiag.x * A) + (diag.y * B) + (offdiag.z * C)) / length);
    ret[3] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z * C)) / length);
}

// run sphere fit to calculate diagonals and offdiagonals
void CompassCalibrator::run_sphere_fit()
{
    if (_sample_buffer == nullptr) {
        return;
    }

    const float lma_damping = 10.0f;

    // take backup of fitness and parameters so we can determine later if this fit has improved the calibration
    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_SPHERE_PARAMS * COMPASS_CAL_NUM_SPHERE_PARAMS] = { };
    float JTJ2[COMPASS_CAL_NUM_SPHERE_PARAMS * COMPASS_CAL_NUM_SPHERE_PARAMS] = { };
    float JTFI[COMPASS_CAL_NUM_SPHERE_PARAMS] = { };

    // Gauss Newton Part common for all kind of extensions including LM
    for (uint16_t k = 0; k < _samples_collected; k++) {
        Vector3f sample = _sample_buffer[k].get();

        float sphere_jacob[COMPASS_CAL_NUM_SPHERE_PARAMS];

        calc_sphere_jacob(sample, fit1_params, sphere_jacob);

        for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
            // compute JTJ
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_SPHERE_PARAMS; j++) {
                JTJ[i * COMPASS_CAL_NUM_SPHERE_PARAMS + j] += sphere_jacob[i] * sphere_jacob[j];
                JTJ2[i * COMPASS_CAL_NUM_SPHERE_PARAMS + j] += sphere_jacob[i] * sphere_jacob[j];   //a backup JTJ for LM
            }
            // compute JTFI
            JTFI[i] += sphere_jacob[i] * calc_residual(sample, fit1_params);
        }
    }

    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    // refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
        JTJ[i * COMPASS_CAL_NUM_SPHERE_PARAMS + i] += _sphere_lambda;
        JTJ2[i * COMPASS_CAL_NUM_SPHERE_PARAMS + i] += _sphere_lambda / lma_damping;
    }

    if (!mat_inverse(JTJ, JTJ, 4)) {
        return;
    }

    if (!mat_inverse(JTJ2, JTJ2, 4)) {
        return;
    }

    // extract radius, offset, diagonals and offdiagonal parameters
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_SPHERE_PARAMS; row++) {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_SPHERE_PARAMS; col++) {
            fit1_params.get_sphere_params()[row] -= JTFI[col] * JTJ[row * COMPASS_CAL_NUM_SPHERE_PARAMS + col];
            fit2_params.get_sphere_params()[row] -= JTFI[col] * JTJ2[row * COMPASS_CAL_NUM_SPHERE_PARAMS + col];
        }
    }

    // calculate fitness of two possible sets of parameters
    fit1 = calc_mean_squared_residuals(fit1_params);
    fit2 = calc_mean_squared_residuals(fit2_params);

    // decide which of the two sets of parameters is best and store in fit1_params
    if (fit1 > _fitness && fit2 > _fitness) {
        // if neither set of parameters provided better results, increase lambda
        _sphere_lambda *= lma_damping;
    }
    else if (fit2 < _fitness && fit2 < fit1) {
        // if fit2 was better we will use it. decrease lambda
        _sphere_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    }
    else if (fit1 < _fitness) {
        fitness = fit1;
    }
    //--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

    // store new parameters and update fitness
    if (!isnan(fitness) && fitness < _fitness) {
        _fitness = fitness;
        _params = fit1_params;
        update_completion_mask();
    }
}

void CompassCalibrator::calc_ellipsoid_jacob(const Vector3f& sample, const param_t& params, float* ret) const
{
    const Vector3f& offset = params.offset;
    const Vector3f& diag = params.diag;
    const Vector3f& offdiag = params.offdiag;
    Matrix3f softiron(
        diag.x, offdiag.x, offdiag.y,
        offdiag.x, diag.y, offdiag.z,
        offdiag.y, offdiag.z, diag.z
    );

    float A = (diag.x * (sample.x + offset.x)) + (offdiag.x * (sample.y + offset.y)) + (offdiag.y * (sample.z + offset.z));
    float B = (offdiag.x * (sample.x + offset.x)) + (diag.y * (sample.y + offset.y)) + (offdiag.z * (sample.z + offset.z));
    float C = (offdiag.y * (sample.x + offset.x)) + (offdiag.z * (sample.y + offset.y)) + (diag.z * (sample.z + offset.z));
    float length = (softiron * (sample + offset)).length();

    // 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
    ret[0] = -1.0f * (((diag.x * A) + (offdiag.x * B) + (offdiag.y * C)) / length);
    ret[1] = -1.0f * (((offdiag.x * A) + (diag.y * B) + (offdiag.z * C)) / length);
    ret[2] = -1.0f * (((offdiag.y * A) + (offdiag.z * B) + (diag.z * C)) / length);
    // 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
    ret[3] = -1.0f * ((sample.x + offset.x) * A) / length;
    ret[4] = -1.0f * ((sample.y + offset.y) * B) / length;
    ret[5] = -1.0f * ((sample.z + offset.z) * C) / length;
    // 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
    ret[6] = -1.0f * (((sample.y + offset.y) * A) + ((sample.x + offset.x) * B)) / length;
    ret[7] = -1.0f * (((sample.z + offset.z) * A) + ((sample.x + offset.x) * C)) / length;
    ret[8] = -1.0f * (((sample.z + offset.z) * B) + ((sample.y + offset.y) * C)) / length;
}

void CompassCalibrator::run_ellipsoid_fit()
{
    if (_sample_buffer == nullptr) {
        return;
    }

    const float lma_damping = 10.0f;

    // take backup of fitness and parameters so we can determine later if this fit has improved the calibration
    float fitness = _fitness;
    float fit1, fit2;
    param_t fit1_params, fit2_params;
    fit1_params = fit2_params = _params;

    float JTJ[COMPASS_CAL_NUM_ELLIPSOID_PARAMS * COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };
    float JTJ2[COMPASS_CAL_NUM_ELLIPSOID_PARAMS * COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };
    float JTFI[COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = { };

    // Gauss Newton Part common for all kind of extensions including LM
    for (uint16_t k = 0; k < _samples_collected; k++) {
        Vector3f sample = _sample_buffer[k].get();

        float ellipsoid_jacob[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];

        calc_ellipsoid_jacob(sample, fit1_params, ellipsoid_jacob);

        for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
            // compute JTJ
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; j++) {
                JTJ[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
                JTJ2[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
            }
            // compute JTFI
            JTFI[i] += ellipsoid_jacob[i] * calc_residual(sample, fit1_params);
        }
    }

    //------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
    //refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
        JTJ[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + i] += _ellipsoid_lambda;
        JTJ2[i * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + i] += _ellipsoid_lambda / lma_damping;
    }

    if (!mat_inverse(JTJ, JTJ, 9)) {
        return;
    }

    if (!mat_inverse(JTJ2, JTJ2, 9)) {
        return;
    }

    // extract radius, offset, diagonals and offdiagonal parameters
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; row++) {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; col++) {
            fit1_params.get_ellipsoid_params()[row] -= JTFI[col] * JTJ[row * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + col];
            fit2_params.get_ellipsoid_params()[row] -= JTFI[col] * JTJ2[row * COMPASS_CAL_NUM_ELLIPSOID_PARAMS + col];
        }
    }

    // calculate fitness of two possible sets of parameters
    fit1 = calc_mean_squared_residuals(fit1_params);
    fit2 = calc_mean_squared_residuals(fit2_params);

    // decide which of the two sets of parameters is best and store in fit1_params
    if (fit1 > _fitness && fit2 > _fitness) {
        // if neither set of parameters provided better results, increase lambda
        _ellipsoid_lambda *= lma_damping;
    }
    else if (fit2 < _fitness && fit2 < fit1) {
        // if fit2 was better we will use it. decrease lambda
        _ellipsoid_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitness = fit2;
    }
    else if (fit1 < _fitness) {
        fitness = fit1;
    }
    //--------------------Levenberg-part-ends-here--------------------------------//

    // store new parameters and update fitness
    if (fitness < _fitness) {
        _fitness = fitness;
        _params = fit1_params;
        update_completion_mask();
    }
}

//////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////CompassSample PUBLIC INTERFACE//////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

#define COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(__X) ((int16_t)constrain_float(roundf(__X*8.0f), INT16_MIN, INT16_MAX))
#define COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(__X) (__X/8.0f)

Vector3f CompassCalibrator::CompassSample::get() const
{
    return Vector3f(COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(x),
        COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(y),
        COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(z));
}

void CompassCalibrator::CompassSample::set(const Vector3f& in)
{
    x = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in.x);
    y = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in.y);
    z = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in.z);
}

/*
  calculate the implied earth field for a compass sample and compass
  rotation. This is used to check for consistency between
  samples.

  If the orientation is correct then when rotated the same (or
  similar) earth field should be given for all samples.

  Note that this earth field uses an arbitrary north reference, so it
  may not match the true earth field.
 */
Vector3f CompassCalibrator::calculate_earth_field(CompassSample& sample, enum Rotation r)
{
    Vector3f v = sample.get();

    // convert the sample back to sensor frame
    v.rotate_inverse(_orientation);

    // rotate to body frame for this rotation
    v.rotate(r);

    // apply offsets, rotating them for the orientation we are testing
    Vector3f rot_offsets = _params.offset;
    rot_offsets.rotate_inverse(_orientation);

    rot_offsets.rotate(r);

    v += rot_offsets;

    // rotate the sample from body frame back to earth frame
    Matrix3f rot = sample.att.get_rotmat();

    Vector3f efield = rot * v;

    // earth field is the mag sample in earth frame
    return efield;
}