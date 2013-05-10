#ifndef SYNTHLAB_FMSINEOSC_HPP
#define SYNTHLAB_FMSINEOSC_HPP

namespace sl {

  class FMSineOsc : public Gen<1,1> {
    static const float PERIOD;

    float phase_;
    float gain_;

  public:
    void play(float gain=1, float phase=0) {
      gain_ = gain;
      phase_ = PERIOD*phase; // phase must be in [0,1]
    }
    void render(int nframes, Sample *freq, Sample *output) {
      int sr = sl::sampleRate();
      for (int i=0; i<nframes; i++) {
	*output++ = gain_*sin(phase_);
        phase_ += PERIOD / (sr / freq[i]);
      }
      phase_ = fmod(phase_, PERIOD);
    }
  };

  const float FMSineOsc::PERIOD = 2*M_PI;

}

#endif
