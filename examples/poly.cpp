#include <unistd.h>

#include <cmath>
#include <cassert>
#include <cstdio>

#include <algorithm>
#include <functional>
#include <string>
#include <iostream>

#include <jack/jack.h>
#include <jack/midiport.h>

namespace sl {

  typedef float Sample;
  typedef int SampleCount;

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

  class SampleBufferAllocator {
    class HeapOverFlowError {};
    static const int HEAPSIZE = 65536;
    static Sample* heap;
    static Sample* pos;
    static Sample* end;
  public:
    static Sample* allocate(int nframes) {
      assert(heap != 0);
      if ((pos+nframes) > end) {
	throw HeapOverFlowError();
      }
      Sample* here = pos;
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

  class SampleBuffer {
    Sample *buffer_;
  public:
    SampleBuffer() :
      buffer_(SampleBufferAllocator::allocate(sl::bufferSize())) {}
    Sample& operator[](const int i) {
      return buffer_[i];
    }
    operator Sample*() {
      return buffer_;
    }
  };

  template <int SIZE>
  class SampleBufferTrunk {
    SampleBuffer buffers_[SIZE];
  public:
    static const int numChannels = SIZE;
    SampleBuffer& operator[](const int i) {
      return buffers_[i];
    }
    void fill(SampleCount nframes, Sample value) {
      for (int i=0; i<SIZE; i++) {
	Sample *buffer = buffers_[i];
	std::fill_n(buffer, nframes, value);
      }
    }
    void add(SampleCount offset, SampleCount size, SampleBufferTrunk &input) {
      for (int i=0; i<SIZE; i++) {
	Sample *src = input[i];
	Sample *dst = buffers_[i];
	std::transform(src,src+size,dst+offset,dst+offset,std::plus<Sample>());
      }
    }
  };

  template <int NUMINPUTS, int NUMOUTPUTS>
  class Generator {
  public:
    typedef SampleBufferTrunk<NUMINPUTS> InputTrunk;
    typedef SampleBufferTrunk<NUMOUTPUTS> OutputTrunk;
  };

  template <class Synth>
  class JackAudioProvider {
    jack_client_t *client_;
    jack_port_t* midiInPort_;
    jack_port_t* inputPorts_[Synth::InputTrunk::numChannels];
    jack_port_t* outputPorts_[Synth::OutputTrunk::numChannels];
    Synth* synth_;

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
      typename Synth::InputTrunk input;
      typename Synth::OutputTrunk output;
      for (int i=0; i<Synth::InputTrunk::numChannels; i++) {
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
	if (midiEvent.size > 2) {
	  unsigned char midiNote = *(midiEvent.buffer+1);
	  unsigned char midiVel = *(midiEvent.buffer+2);
	  SampleCount delay = midiEvent.time;
	  switch (midiCommand) {
	  case 0x90:
	    synth->noteOn(midiNote, midiVel, delay);
	    break;
	  case 0x80:
	    synth->noteOff(midiNote, midiVel, delay);
	    break;
	  }
	}
      }
      synth->render(nframes, input, output);
      for (int i=0; i<Synth::OutputTrunk::numChannels; i++) {
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
      for (int i=0; i<Synth::InputTrunk::numChannels; i++) {
	snprintf(portName, sizeof(portName), "in_%u", i);
	inputPorts_[i] = jack_port_register(client_, portName, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
      }
      for (int i=0; i<Synth::OutputTrunk::numChannels; i++) {
	snprintf(portName, sizeof(portName), "out_%u", i);
	outputPorts_[i] = jack_port_register(client_, portName, JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0);
      }
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

  struct VoiceInfo {
    bool active;
    SampleCount delay;
    unsigned char midiNote;
    unsigned char midiVel;
    VoiceInfo()
      : active(false),
	delay(0),
	midiNote(0),
	midiVel(0) {}
  };

  template <class Voice,
	    int MAXVOICES=16,
	    template <class Synth> class AudioProvider = JackAudioProvider>
  class PolySynth {
  public:
    typedef typename Voice::InputTrunk InputTrunk;
    typedef typename Voice::OutputTrunk OutputTrunk;
    PolySynth(const std::string &name)
      : audioProvider_(this, name)
    {
    }
    ~PolySynth() {
      stop();
    }
    void render(SampleCount nframes, InputTrunk &input, OutputTrunk &output) {
      output.fill(nframes, 0);
      OutputTrunk o;
      for (int i=0; i<MAXVOICES; i++) {
	if (voiceInfo_[i].active) {
	  SampleCount delay = voiceInfo_[i].delay;
	  voiceInfo_[i].active = voices_[i].render(nframes-delay, input, o);
	  output.add(delay, nframes-delay, o);
	  voiceInfo_[i].delay = 0;
	}
      }
    }
    void noteOn(unsigned char midiNote, unsigned char midiVel, SampleCount delay) {
      // std::cerr << "noteOn(" << int(midiNote) << ", " << int(midiVel) << ", " << delay << ")" << std::endl;
      for (int i=0; i<MAXVOICES; i++) {
	if (voiceInfo_[i].active) continue;
	voices_[i].play(midiNote, midiVel);
	voiceInfo_[i].midiNote = midiNote;
	voiceInfo_[i].midiVel = midiVel;
	voiceInfo_[i].delay = delay;
	voiceInfo_[i].active = true;
	break;
      }
    }
    void noteOff(unsigned char midiNote, unsigned char midiVel, SampleCount delay) {
      // std::cerr << "noteOff(" << int(midiNote) << ", " << int(midiVel) << ", " << delay << ")" << std::endl;
      for (int i=0; i<MAXVOICES; i++) {
	if (! voiceInfo_[i].active) continue;
	if (voiceInfo_[i].midiNote != midiNote) continue;
	voices_[i].release();
      }
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
    AudioProvider<PolySynth> audioProvider_;
  };

  class SineOsc : public Generator<0,1> {
    float freq_;
    float phase_;
    float step_;

    void calcStep() {
      step_ = (2*M_PI) / (sl::sampleRate() / freq_);
    }

  public:
    void setFreq(float freq) {
      freq_ = freq;
      calcStep();
    }
    void play(float freq) {
      setFreq(freq);
      phase_ = 0;
    }
    void render(SampleCount nframes, OutputTrunk &output) {
      Sample *buf = output[0];
      for (int i=0; i<nframes; i++) {
	*buf++ = sin(phase_);
	phase_ += step_;
      }
      phase_ = fmod(phase_, 2*M_PI);
    }
  };
}

float midi2cps(unsigned char midiNote) {
  return 440.0 * exp(log(2.0)*((midiNote-69)/12.0));
}

class Voice : public sl::Generator<0,1> {
  sl::SineOsc osc_;
  bool playing_;
public:
  Voice() :
    playing_(false)
  {
  }
  bool render(sl::SampleCount nframes, InputTrunk &input, OutputTrunk &output) {
    osc_.render(nframes, output);
    return playing_;
  }
  void play(unsigned char midiNote, unsigned char midiVel) {
    osc_.play(midi2cps(midiNote));
    playing_ = true;
  }
  void release() {
    playing_ = false;
  }
};

int main(int argc, char **argv) {
  sl::PolySynth<Voice> synth("poly");
  synth.start();
  while (true) {
    sleep(1);
  }
}
