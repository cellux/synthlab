#include <synthlab.hpp>

struct VoiceData {
  sl::Env env;
  float pb; // freq multiplier
  VoiceData() : pb(1.0) {
    env.add(sl::Env::Set(0));
    env.add(sl::Env::Slide(1.0,0.2));
    env.add(sl::Env::Slide(0.7,0.1));
    env.add(sl::Env::Sustain());
    env.add(sl::Env::Slide(0.0,1.0));
  }
  void cc(int num, int value) {
    //std::cout << "num=" << num << ", value=" << value << "\n";
    switch (num) {
    case 1:
      // attack time
      env[1].setTime((5*value)/127.0);
      break;
    case 2:
      // decay time
      env[2].setTime(value/127.0);
      break;
    case 3:
      // sustain level
      env[2].setTarget(value/127.0);
      break;
    case 4:
      // release time
      env[4].setTime((5*value)/127.0);
      break;
    case 128:
      // pitch bend
      pb = pow(2,((float)value-0x2000)/8192.0);
      break;
    }
  }
};

class Voice : public sl::Gen<0,2,VoiceData> {
  sl::SineOsc osc_;
  sl::Env env_;
  float gain_;
  float basefreq_;
  float pb_;
public:
  void play(int midiNote, int midiVel) {
    // each voice gets its own copy of the shared env
    //
    // => control changes don't affect voices already playing
    env_ = data_->env;
    basefreq_ = sl::midi2cps(midiNote);
    pb_ = data_->pb;
    osc_.play(basefreq_ * pb_);
    env_.play();
    gain_ = midiVel / 127.0;
  }
  bool render(int nframes, OutputBuffer &output, InputBuffer &input) {
    if (pb_ != data_->pb) {
      pb_ = data_->pb;
      osc_.setFreq(basefreq_ * pb_);
    }
    sl::SampleBuffer<1> osc_output;
    osc_.render(nframes, osc_output[0]);
    osc_output.mul(nframes, gain_);
    sl::SampleBuffer<1> env_output;
    bool playing = env_.render(nframes, env_output[0]);
    osc_output.mul(nframes, env_output);
    std::copy(osc_output[0], osc_output[0]+nframes, output[0]);
    std::copy(osc_output[0], osc_output[0]+nframes, output[1]);
    return playing;
  }
  void release(int delay) {
    env_.release(delay);
  }
};

int main(int argc, char **argv) {
  sl::PolySynth<Voice> synth("poly");
  synth.start();
  getchar();
}
