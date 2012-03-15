#include <dope/sl.h>

template <int NUMOSC>
class Voice : public sl::Generator<0,1> {
  sl::SineOsc oscbank_[NUMOSC];
  sl::Env env_;
  float gain_;
  bool playing_;
public:
  Voice() :
    gain_(0),
    playing_(false)
  {
    env_.add(sl::Env::Set(0));
    env_.add(sl::Env::Slide(1.0,3.0));
    env_.add(sl::Env::Sustain());
    env_.add(sl::Env::Slide(0.0,3.0));
  }
  bool render(sl::SampleCount nframes, InputTrunk &input, OutputTrunk &output) {
    output.fill(nframes, 0);
    OutputTrunk osc_output;
    for (int i=0; i<NUMOSC; i++) {
      oscbank_[i].render(nframes, osc_output);
      output.add(nframes, osc_output);
    }
    output.mul(nframes, gain_);
    OutputTrunk env_output;
    playing_ = env_.render(nframes, env_output);
    output.mul(nframes, env_output);
    return playing_;
  }
  void play(unsigned char midiNote, unsigned char midiVel) {
    float f = sl::midi2cps(midiNote);
    for (int i=0; i<NUMOSC; i++, f*=2.0) {
      oscbank_[i].play(f,(rand() % 10)/(10.0*NUMOSC));
    }
    env_.play();
    gain_ = midiVel / 127.0;
    playing_ = true;
  }
  void release(sl::SampleCount delay) {
    env_.release(delay);
  }
};

int main(int argc, char **argv) {
  sl::PolySynth< Voice<16> > synth("additive");
  synth.start();
  while (true) {
    sleep(1);
  }
}
