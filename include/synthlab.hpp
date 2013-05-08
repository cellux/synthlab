#include <stdio.h>
#include <unistd.h>

#include <cmath>
#include <cassert>
#include <cstdio>

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <functional>
#include <numeric>

#include <jack/jack.h>
#include <jack/midiport.h>

namespace sl {

  typedef float Sample;

  inline int sampleRate(int sr = 0) {
    static int sr_ = 0;
    if (sr != 0) sr_ = sr;
    return sr_;
  }

  inline int bufferSize(int bs = 0) {
    static int bs_ = 0;
    if (bs != 0) bs_ = bs;
    return bs_;
  }

  template <class T> struct AbsCmp : std::binary_function<T,T,T> {
    T operator() (const T &x, const T &y) const {
      return (fabs(x) < fabs(y));
    }
  };

  template <class Iter>
  Sample peakAmplitude(Iter first, Iter last) {
    typedef typename std::iterator_traits<Iter>::value_type value_type;
    return fabs(*std::max_element(first, last, AbsCmp<value_type>()));
  }

  template <class T> struct MeanPowerAcc : std::binary_function<T,T,T> {
    T operator() (const T &x, const T &y) const {
      return x+y*y;
    }
  };

  template <class Iter>
  Sample meanPower(Iter first, Iter last) {
    typedef typename std::iterator_traits<Iter>::value_type value_type;
    if (first == last) {
      return 0;
    }
    else {
      return std::accumulate(first, last, 0, MeanPowerAcc<value_type>())/(last-first);
    }
  }

  template <class Iter>
  Sample rmsAmplitude(Iter first, Iter last) {
    return sqrt(meanPower(first, last));
  }

  const float dbRefAmplitude = 0.00001;

  float amp2db(float amp) {
    return 20*log10(amp/dbRefAmplitude);
  }

  float midi2cps(unsigned char midiNote) {
    return 440.0 * pow(2.0, (midiNote-69)/12.0);
  }

  unsigned char cps2midi(float cps) {
    return 69 + 12*log2(cps/440);
  }

  class SampleBufferAllocator {
    class HeapOverFlowError {};
    // increase HEAPSIZE if you get HeapOverFlowError
    static const int HEAPSIZE = 65536;
    static Sample *heap;
    static Sample *pos;
    static Sample *end;
  public:
    static Sample* allocate(int nframes) {
      assert(heap != 0);
      if ((pos+nframes) > end) {
	throw HeapOverFlowError();
      }
      Sample *here = pos;
      pos += nframes;
      return here;
    }
    static void reset() {
      if (heap == 0) {
	heap = new Sample[HEAPSIZE];
	end = heap+HEAPSIZE;
      }
      pos = heap;
    }
  };

  Sample* SampleBufferAllocator::heap = 0;
  Sample* SampleBufferAllocator::pos = 0;
  Sample* SampleBufferAllocator::end = 0;

  template <int NCHANNELS>
  class SampleBuffer {
    int stride_;
    Sample *buffers_;
  public:
    static const int nChannels = NCHANNELS;
    SampleBuffer() :
      stride_(sl::bufferSize()),
      buffers_(SampleBufferAllocator::allocate(stride_*NCHANNELS)) {}
    Sample* operator[](const int i) {
      return buffers_+stride_*i;
    }
    void fill(int nframes, Sample value) {
      Sample *dst = buffers_;
      for (int i=0; i<NCHANNELS; i++) {
	std::fill_n(dst, nframes, value);
	dst += stride_;
      }
    }
    template <class BinaryOp>
    void transform(int nframes,
		   SampleBuffer &input,
		   BinaryOp op,
		   int offset)
    {
      Sample *dst = buffers_;
      for (int i=0; i<NCHANNELS; i++) {
	Sample *src = input[i];
	std::transform(src,src+nframes,dst+offset,dst+offset,op);
	dst += stride_;
      }
    }
    void add(int nframes, SampleBuffer &input, int offset=0) {
      transform(nframes, input, std::plus<Sample>(), offset);
    }
    void mul(int nframes, SampleBuffer &input, int offset=0) {
      transform(nframes, input, std::multiplies<Sample>(), offset);
    }
    template <class UnaryOp>
    void transform(int nframes,
		   UnaryOp op,
		   int offset)
    {
      Sample *dst = buffers_;
      for (int i=0; i<NCHANNELS; i++) {
	std::transform(dst+offset,dst+nframes,dst+offset,op);
	dst += stride_;
      }
    }
    void add(int nframes, const Sample value, int offset=0) {
      transform(nframes, std::bind2nd(std::plus<Sample>(), value), offset);
    }
    void mul(int nframes, const Sample value, int offset=0) {
      transform(nframes, std::bind2nd(std::multiplies<Sample>(), value), offset);
    }
  };

  class NoData {
  public:
    void cc(int num, int value) {}
  };

  template <int NUMINPUTS, int NUMOUTPUTS, class GenData=NoData>
  class Gen {
  protected:
    GenData *data_;
  public:
    typedef GenData Data;
    void data(GenData *data) { data_ = data; }
    typedef SampleBuffer<NUMINPUTS> InputBuffer;
    typedef SampleBuffer<NUMOUTPUTS> OutputBuffer;
  };

  template <class Synth>
  class JackAudioProvider {
    jack_client_t *client_;
    jack_port_t* midiInPort_;
    jack_port_t* inputPorts_[Synth::InputBuffer::nChannels];
    jack_port_t* outputPorts_[Synth::OutputBuffer::nChannels];
    Synth* synth_;
    unsigned char midiCtrlMap_[128];

    static int jackSampleRateCallback(jack_nframes_t sr, void *arg) {
      sl::sampleRate(sr);
      return 0;
    }

    static int jackBufferSizeCallback(jack_nframes_t bs, void *arg) {
      sl::bufferSize(bs);
      return 0;
    }

    static int jackProcessCallback(jack_nframes_t nframes, void *arg) {
      JackAudioProvider *jap = static_cast<JackAudioProvider*>(arg);
      Synth *synth = jap->synth_;
      SampleBufferAllocator::reset();
      typename Synth::InputBuffer input;
      typename Synth::OutputBuffer output;
      for (int i=0; i<Synth::InputBuffer::nChannels; i++) {
	jack_default_audio_sample_t *in = (jack_default_audio_sample_t*) jack_port_get_buffer(jap->inputPorts_[i], nframes);
	Sample *buf = input[i];
	std::copy(in,in+nframes,buf);
      }
      void *midiBuf = jack_port_get_buffer(jap->midiInPort_, nframes);
      jack_nframes_t midiEventCount = jack_midi_get_event_count(midiBuf);
      jack_midi_event_t midiEvent;
      for (int i=0; i<midiEventCount; i++) {
	jack_midi_event_get(&midiEvent, midiBuf, i);
	unsigned char midiCommand = *(midiEvent.buffer) & 0xf0;
        //std::cout << "got midi command: " << int(midiCommand) << "\n";
	if (midiEvent.size > 2) {
	  unsigned char midiValue1 = *(midiEvent.buffer+1);
	  unsigned char midiValue2 = *(midiEvent.buffer+2);
	  int delay = midiEvent.time;
	  switch (midiCommand) {
	  case 0x80:
	    synth->noteOff(midiValue1, midiValue2, delay);
	    break;
	  case 0x90:
	    synth->noteOn(midiValue1, midiValue2, delay);
	    break;
          case 0xb0:
            synth->controlChange(jap->midiCtrlMap_[midiValue1], midiValue2);
            break;
          case 0xe0:
            // pitch bend
            synth->controlChange(128, midiValue1 | (midiValue2<<7));
            break;
	  }
	}
      }
      synth->render(nframes, output, input);
      for (int i=0; i<Synth::OutputBuffer::nChannels; i++) {
	jack_default_audio_sample_t *out = (jack_default_audio_sample_t*) jack_port_get_buffer(jap->outputPorts_[i], nframes);
	Sample *buf = output[i];
	std::copy(buf,buf+nframes,out);
      }
      return 0;
    }

  public:
    // exceptions
    class JackClientOpenError {};
    class JackActivateError {};
    class JackDeactivateError {};

    JackAudioProvider(Synth *synth, const std::string &name)
      : synth_(synth)
    {
      SampleBufferAllocator::reset();
      client_ = jack_client_open(name.c_str(), JackNoStartServer, 0);
      if (client_ == NULL) {
	throw JackClientOpenError();
      }
      sl::sampleRate(jack_get_sample_rate(client_));
      jack_set_sample_rate_callback(client_, jackSampleRateCallback, 0);
      sl::bufferSize(jack_get_buffer_size(client_));
      jack_set_buffer_size_callback(client_, jackBufferSizeCallback, 0);
      jack_set_process_callback(client_, jackProcessCallback, this);
      midiInPort_ = jack_port_register(client_, "midi_in", JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0);
      char portName[64];
      for (int i=0; i<Synth::InputBuffer::nChannels; i++) {
	snprintf(portName, sizeof(portName), "in_%u", i+1);
	inputPorts_[i] = jack_port_register(client_, portName, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
      }
      for (int i=0; i<Synth::OutputBuffer::nChannels; i++) {
	snprintf(portName, sizeof(portName), "out_%u", i+1);
	outputPorts_[i] = jack_port_register(client_, portName, JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
      }
      for (int i=0; i<128; i++) {
        midiCtrlMap_[i] = i;
      }
      for (int i=11; i<=19; i++) {
        midiCtrlMap_[i] = i-10;
      }
      midiCtrlMap_[1] = 129; // modwheel
    }

    ~JackAudioProvider() {
      if (client_ != 0) {
	jack_client_close(client_);
      }
    }

    void start() {
      if (jack_activate(client_) != 0) {
	throw JackActivateError();
      }
    }

    void stop() {
      if (jack_deactivate(client_) != 0) {
	throw JackDeactivateError();
      }
    }
  };

  enum VoiceStatus {
    VS_PLAYING,
    VS_RELEASING,
    VS_INACTIVE
  };

  struct VoiceInfo {
    VoiceStatus status;
    int delay;
    unsigned char midiNote;
    unsigned char midiVel;
    VoiceInfo()
      : status(VS_INACTIVE),
	delay(0),
	midiNote(0),
	midiVel(0) {}
  };

  template <class Voice,
	    int MAXVOICES=16,
	    template <class Synth> class AudioProvider = JackAudioProvider>
  class PolySynth {
  public:
    typedef typename Voice::InputBuffer InputBuffer;
    typedef typename Voice::OutputBuffer OutputBuffer;
    typedef typename Voice::Data VoiceData;
    PolySynth(const std::string &name)
      : audioProvider_(this, name)
    {
      for (int i=0; i<MAXVOICES; i++) {
	voices_[i].data(&voiceData_);
      }
    }
    ~PolySynth() {
      stop();
    }
    void render(int nframes, OutputBuffer &output, InputBuffer &input) {
      output.fill(nframes, 0);
      OutputBuffer o;
      for (int i=0; i<MAXVOICES; i++) {
	if (voiceInfo_[i].status != VS_INACTIVE) {
	  int delay = voiceInfo_[i].delay;
	  bool voiceStillActive = voices_[i].render(nframes-delay, o, input);
          if (! voiceStillActive) {
            voiceInfo_[i].status = VS_INACTIVE;
          }
	  output.add(nframes-delay, o, delay); // nframes, input, offset
	  voiceInfo_[i].delay = 0;
	}
      }
    }
    void noteOn(int midiNote, int midiVel, int delay) {
      int inactive = -1;
      for (int i=0; i<MAXVOICES; i++) {
        if (inactive == -1 && voiceInfo_[i].status == VS_INACTIVE) inactive = i;
        if (voiceInfo_[i].midiNote == midiNote && voiceInfo_[i].status == VS_PLAYING) {
          voices_[i].release(delay);
          voiceInfo_[i].status = VS_RELEASING;
        }
      }
      if (inactive >= 0) {
	voices_[inactive].play(midiNote, midiVel);
	voiceInfo_[inactive].midiNote = midiNote;
	voiceInfo_[inactive].midiVel = midiVel;
	voiceInfo_[inactive].delay = delay;
	voiceInfo_[inactive].status = VS_PLAYING;
      }
    }
    void noteOff(int midiNote, int midiVel, int delay) {
      for (int i=0; i<MAXVOICES; i++) {
	if (voiceInfo_[i].status != VS_PLAYING) continue;
	if (voiceInfo_[i].midiNote != midiNote) continue;
	voices_[i].release(delay);
        voiceInfo_[i].status = VS_RELEASING;
	break;
      }
    }
    void controlChange(int num, int value) {
      voiceData_.cc(num, value);
    }
    void start() {
      audioProvider_.start();
    }
    void stop() {
      audioProvider_.stop();
    }

  private:
    Voice voices_[MAXVOICES];
    VoiceInfo voiceInfo_[MAXVOICES];
    VoiceData voiceData_;
    AudioProvider<PolySynth> audioProvider_;
  };

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

  class Env : public Gen<0,1> {
    enum EnvCommandType {
      SET,
      SLIDE,
      SUSTAIN
    };
    struct EnvCommand {
      EnvCommandType type;
      union {
	struct {
	  Sample value;
	} set;
	struct {
	  Sample target;
	  float time;
	} slide;
      };
      void setValue(Sample value) {
        set.value = value;
      }
      void setTarget(Sample target) {
        slide.target = target;
      }
      void setTime(float time) {
        slide.time = time;
      }
    };
    typedef std::vector<EnvCommand> EnvCommandVec;
    EnvCommandVec commands_;
    EnvCommandVec::const_iterator ip_;
    Sample value_;
    Sample increment_;
    Sample length_;
  public:
    class BadEnvCommandType {};

    Env()
      : commands_(),
	ip_(commands_.begin()),
	value_(0),
	increment_(0),
	length_(0)
    {
    }
    static EnvCommand Set(Sample value) {
      EnvCommand cmd;
      cmd.type = SET;
      cmd.set.value = value;
      return cmd;
    }
    static EnvCommand Slide(Sample target, float time) {
      EnvCommand cmd;
      cmd.type = SLIDE;
      cmd.slide.target = target;
      cmd.slide.time = time;
      return cmd;
    }
    static EnvCommand Sustain() {
      EnvCommand cmd;
      cmd.type = SUSTAIN;
      return cmd;
    }
    bool render(int nframes, Sample *output) {
      if (length_ == -1) {
	// sustained for the entire block
	std::fill_n(output,nframes,value_);
	return true;
      }
      for (int i=0; i<nframes; i++) {
      STEP:
	if (length_ > 0) {
	  output[i] = value_;
	  value_ += increment_;
	  --length_;
	}
	else {
	CMD:
	  if (ip_ == commands_.end()) {
	    std::fill(output+i,output+nframes,0);
	    return false;
	  }
	  else {
	    const EnvCommand &cmd = *ip_++;
	    switch (cmd.type) {
	    case SET:
	      value_ = cmd.set.value;
	      goto CMD;
	    case SLIDE:
	      length_ = sl::sampleRate()*cmd.slide.time;
	      increment_ = (cmd.slide.target - value_) / length_;
	      goto STEP;
	    case SUSTAIN:
	      length_ = -1;
	      increment_ = 0;
	      std::fill(output+i,output+nframes,value_);
	      return true;
	    default:
	      throw BadEnvCommandType();
	    }
	  }
	}
      }
      return true;
    }
    void add(EnvCommand cmd) {
      commands_.push_back(cmd);
    }
    void play() {
      ip_ = commands_.begin();
      length_ = 0;
      value_ = 0;
      increment_ = 0;
      length_ = 0;
    }
    void release(int delay) {
      if (length_ == -1) {
	// we are sustaining the note
	// advance to next command after `delay' samples
	length_ = delay;
      }
      else {
	// look for next sustain command
	EnvCommandVec::const_iterator pos;
	for (pos=ip_; pos != commands_.end(); ++pos) {
	  const EnvCommand &cmd = *pos;
	  if (cmd.type == SUSTAIN) break;
	}
	if (pos != commands_.end()) {
	  // found: jump to command following it
	  ip_ = pos+1;
	  length_ = delay;
	}
      }
    }
    EnvCommand& operator[](const int i) {
      return commands_[i];
    }
    Env& operator=(const Env &other) {
      if (this != &other) {
        commands_ = other.commands_;
      }
      return *this;
    }
  };
}
