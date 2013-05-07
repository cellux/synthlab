#include <synthlab.hpp>

template <int NUMOSC>
class Voice : public sl::Gen<0,1> {
  sl::SineOsc oscbank_[NUMOSC];
  sl::Env env_;
  float gain_;
public:
  Voice() :
    gain_(0)
  {
    env_.add(sl::Env::Set(0));
    env_.add(sl::Env::Slide(1.0,3.0));
    env_.add(sl::Env::Sustain());
    env_.add(sl::Env::Slide(0.0,3.0));
  }
  void play(int midiNote, int midiVel) {
    float f = sl::midi2cps(midiNote);
    for (int i=0; i<NUMOSC; i++, f*=2.0) {
      oscbank_[i].play(f,(rand() % 10)/(10.0*NUMOSC));
    }
    env_.play();
    gain_ = midiVel / 127.0;
  }
  bool render(int nframes, OutputBuffer &output, InputBuffer &input) {
    output.fill(nframes, 0);
    OutputBuffer osc_output;
    for (int i=0; i<NUMOSC; i++) {
      oscbank_[i].render(nframes, osc_output[0]);
      output.add(nframes, osc_output);
    }
    output.mul(nframes, gain_);
    sl::SampleBuffer<1> env_output;
    bool playing = env_.render(nframes, env_output[0]);
    output.mul(nframes, env_output);
    return playing;
  }
  void release(int delay) {
    env_.release(delay);
  }
};

int main(int argc, char **argv) {
  sl::PolySynth< Voice<16> > synth("additive");
  synth.start();
  getchar();
}
