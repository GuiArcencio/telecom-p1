#include <math.h>
#include <numbers>
#include "v21.hpp"

constexpr float BANDPASS_SMOOTHING = 0.99;
constexpr float CLOCKFILTER_SMOOTHING = 0.9999;
constexpr int CLOCKFILTER_DELAY = 40;

V21_RX::V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples) :
    omega_mark(omega_mark),
    omega_space(omega_space),
    get_digital_samples(get_digital_samples) 
{
    // Filling buffers with "empty" values
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++)
        this->sample_buffer.push_front(0.f);
    for (int i = 0; i < CLOCKFILTER_DELAY; i++)
        this->decision_buffer.push_front(0.f);
    this->vspace_r_buffer = 0.f;
    this->vspace_i_buffer = 0.f;
    this->vmark_r_buffer = 0.f;
    this->vmark_i_buffer = 0.f;
    this->clock_filter_buffer[0] = 0.f;
    this->clock_filter_buffer[1] = 0.f;
    this->last_clock = 0.f;
    this->clock_sample_buffer = 0.f;
    this->clock_sample_count = 0;

    this->high_digital_samples = new unsigned int[SAMPLES_PER_SYMBOL];
    this->low_digital_samples = new unsigned int[SAMPLES_PER_SYMBOL];
    for (int i = 0; i < SAMPLES_PER_SYMBOL; i++) {
        this->high_digital_samples[i] = 1;
        this->low_digital_samples[i] = 0;
    }

    // Precomputing sines and cosines
    this->rl_cos_space =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * cos(omega_space * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->rl_sin_space =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * sin(omega_space * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->r_cos_space = BANDPASS_SMOOTHING * cos(omega_space * SAMPLING_PERIOD); 
    this->r_sin_space = BANDPASS_SMOOTHING * sin(omega_space * SAMPLING_PERIOD); 

    this->rl_cos_mark =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * cos(omega_mark * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->rl_sin_mark =
        pow(BANDPASS_SMOOTHING, SAMPLES_PER_SYMBOL)
        * sin(omega_mark * SAMPLES_PER_SYMBOL * SAMPLING_PERIOD); 
    this->r_cos_mark = BANDPASS_SMOOTHING * cos(omega_mark * SAMPLING_PERIOD); 
    this->r_sin_mark = BANDPASS_SMOOTHING * sin(omega_mark * SAMPLING_PERIOD); 
};

V21_RX::~V21_RX() {
    delete[] this->high_digital_samples;
    delete[] this->low_digital_samples;
}

void V21_RX::demodulate(const float *in_analog_samples, unsigned int n)
{
    const int L = SAMPLES_PER_SYMBOL;

    for (int i = 0; i < n; i++) {
        this->sample_buffer.push_front(in_analog_samples[i]);

        float vspace_r = this->sample_buffer[0] - this->rl_cos_space * this->sample_buffer[L]
                            + this->r_cos_space * this->vspace_r_buffer - this->r_sin_space * vspace_i_buffer;
        float vspace_i = -this->rl_sin_space * this->sample_buffer[L] + this->r_cos_space * vspace_i_buffer
                            + this->r_sin_space * vspace_r_buffer;
        float vmark_r = this->sample_buffer[0] - this->rl_cos_mark * this->sample_buffer[L]
                            + this->r_cos_mark * this->vmark_r_buffer - this->r_sin_mark * vmark_i_buffer;
        float vmark_i = -this->rl_sin_mark * this->sample_buffer[L] + this->r_cos_mark * vmark_i_buffer
                            + this->r_sin_mark * vmark_r_buffer;

        float decision = vmark_r * vmark_r + vmark_i * vmark_i -
                                vspace_r * vspace_r - vspace_i * vspace_i; 
        this->decision_buffer.push_front(decision);

        this->clock_sample_buffer += this->decision_buffer[CLOCKFILTER_DELAY];
        this->clock_sample_count++;

        float c = abs(decision);
        float clock_filter = 
            (1 - CLOCKFILTER_SMOOTHING) * c 
            + 2 * CLOCKFILTER_SMOOTHING * cos((2.f * M_PI * BAUD_RATE) / SAMPLING_RATE) * this->clock_filter_buffer[0]
            - CLOCKFILTER_SMOOTHING * CLOCKFILTER_SMOOTHING * this->clock_filter_buffer[1];
        float clock_val = clock_filter - this->clock_filter_buffer[1];

        if (last_clock < 0.f && clock_val >= 0.f) {
            // Symbol frontier
            this->clock_sample_buffer /= this->clock_sample_count;
            
            if (this->clock_sample_buffer < 0.f)
                get_digital_samples(this->low_digital_samples, L);
            else
                get_digital_samples(this->high_digital_samples, L);

            this->clock_sample_buffer = 0.f;
            this->clock_sample_count = 0;
        }

        // Updating buffers 
        this->sample_buffer.pop_back();
        this->decision_buffer.pop_back();
        this->vspace_r_buffer = vspace_r;
        this->vspace_i_buffer = vspace_i;
        this->vmark_r_buffer = vmark_r;
        this->vmark_i_buffer = vmark_i;
        this->clock_filter_buffer[1] = this->clock_filter_buffer[0];
        this->clock_filter_buffer[0] = clock_filter;
        this->last_clock = clock_val;
    }
}

void V21_TX::modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n)
{
    while (n--) {
        *out_analog_samples++ = sin(phase);
        phase += (*in_digital_samples++ ? omega_mark : omega_space) * SAMPLING_PERIOD;

        // evita que phase cresça indefinidamente, o que causaria perda de precisão
        phase = remainder(phase, 2*std::numbers::pi);
    }
}
