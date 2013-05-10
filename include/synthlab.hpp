#ifndef SYNTHLAB_HPP
#define SYNTHLAB_HPP

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
    if (sr != 0 && sr != sr_) {
      sr_ = sr;
      std::cout << "sampleRate=" << sr << "\n";
    }
    return sr_;
  }

  inline int bufferSize(int bs = 0) {
    static int bs_ = 0;
    if (bs != 0 && bs != bs_) {
      bs_ = bs;
      std::cout << "bufferSize=" << bs << "\n";
    }
    return bs_;
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
    SampleBuffer(int stride = sl::bufferSize(), Sample *buffers = 0) :
      stride_(stride),
      buffers_(buffers == 0 ? SampleBufferAllocator::allocate(stride_*NCHANNELS) : buffers)
    {}
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

  template <int NCHANNELS>
  class StaticSampleBuffer : public SampleBuffer<NCHANNELS> {
  public:
    StaticSampleBuffer(int size = sl::bufferSize()) :
      SampleBuffer<NCHANNELS>(size, new Sample[size*NCHANNELS])
    {}
  };

  class NoData {
  public:
    void cc(int num, int value) {}
    void render(int nframes) {}
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

}

#include <synthlab/util.hpp>
#include <synthlab/audioproviders/JackAudioProvider.hpp>
#include <synthlab/PolySynth.hpp>

#include <synthlab/generators/SlidingValue.hpp>
#include <synthlab/generators/Env.hpp>

#include <synthlab/generators/SineOsc.hpp>
#include <synthlab/generators/FMSineOsc.hpp>
#include <synthlab/generators/Noise.hpp>

#endif
