#ifndef SYNTHLAB_SINEOSC_HPP
#define SYNTHLAB_SINEOSC_HPP

namespace sl {

  class SineOsc : public Gen<0,1> {
    static const float PERIOD;

    float freq_;
    float phase_;
    float gain_;
    float step_;

    void calcStep() {
      step_ = PERIOD / (sl::sampleRate() / freq_);
    }

  public:
    void setFreq(float freq) {
      freq_ = freq;
      calcStep();
    }
    void play(float freq, float gain=1, float phase=0) {
      setFreq(freq);
      gain_ = gain;
      phase_ = PERIOD*phase; // phase must be in [0,1]
    }
    void render(int nframes, Sample *output) {
      for (int i=0; i<nframes; i++) {
	*output++ = gain_*sin(phase_);
	phase_ += step_;
      }
      phase_ = fmod(phase_, PERIOD);
    }
  };

  const float SineOsc::PERIOD = 2*M_PI;

}

#endif
