#ifndef V21_HPP
#define V21_HPP

#include <functional>
#include <deque>
#include "config.hpp"

class V21_RX
{
public:
    V21_RX(float omega_mark, float omega_space, std::function<void(const unsigned int *, unsigned int)> get_digital_samples);
    ~V21_RX();
    void demodulate(const float *in_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float rl_cos_space, rl_sin_space, r_cos_space, r_sin_space; 
    float rl_cos_mark, rl_sin_mark, r_cos_mark, r_sin_mark; 

    std::deque<float> sample_buffer;
    float vspace_r_buffer, vspace_i_buffer;
    float vmark_r_buffer, vmark_i_buffer;
    float clock_filter_buffer[2];
    float last_clock;
    std::deque<float> decision_buffer;
    float clock_sample_buffer;
    int clock_sample_count;

    unsigned int *high_digital_samples, *low_digital_samples;

    std::function<void(const unsigned int *, unsigned int)> get_digital_samples;
};

class V21_TX
{
public:
    V21_TX(float omega_mark, float omega_space) :omega_mark(omega_mark),omega_space(omega_space),phase(0.f) {};
    void modulate(const unsigned int *in_digital_samples, float *out_analog_samples, unsigned int n);
private:
    float omega_mark, omega_space;
    float phase;
};

#endif
