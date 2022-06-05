
Skip to content
Pull requests
Issues
Marketplace
Explore
@mantasnaris
lab-collab /
srtx_firmware
Private

Code
Issues 1
Pull requests
Actions
Projects
Wiki
Security
Insights

    Settings

srtx_firmware/lib/SRTxMCU/CircularBuffer.h
@mantasnaris
mantasnaris compiles, runs, and works!
Latest commit 59d9c67 on Jun 23, 2021
History
1 contributor
151 lines (117 sloc) 2.74 KB
#ifndef H_CIRCBUFFER
#define H_CIRCBUFFER
/* A circular buffer template class implementation, which can store data of any type needed
modified code from source: https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp
more info: https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
usage instructions (copied & modified from link above):
    To instantiate a circular buffer, we just declare an object and specify the templated type for our buffer. Here’s an example using a buffer of 10 uint32_t entries:
        CircularBuffer<uint32_t> buffer(10);
    Adding data is easy:
        uint32_t x = 100;
        buffer.put(x);
    And getting data is equally easy:
        x = buffer.pop(); // removes value
		x = buffer.peek(); // does not remove value
	extra feature:
		x = buffer.pad(); // fills with zeros up to BLOCK_SIZE
*/

#include <cstdio>
#include <memory>
#include <Arduino.h>


template <class T>
class CircularBuffer {
public:
	explicit CircularBuffer(size_t size) :
		buf_(std::unique_ptr<T[]>(new T[size])),
		max_size_(size)
	{

	}

	void push(T item) {

		buf_[head_] = item;

		if(full_) {
			tail_ = (tail_ + 1) % max_size_;
			// Serial.println(F("buffer overflow"));
		}

		head_ = (head_ + 1) % max_size_;

		full_ = head_ == tail_;
	}

	T pop()
	{
		if(empty())
		{
			return T();
		}

		//Read data and advance the tail (we now have a free space)
		auto val = buf_[tail_];
		full_ = false;
		tail_ = (tail_ + 1) % max_size_;

		return val;
	}

	//FUTURE: add offset as for peek
	void del()
	{
		if(empty())
		{
			return;
		}

		//advance the tail (we now have a free space)
		full_ = false;
		tail_ = (tail_ + 1) % max_size_;

		return;
	}

	//read the value at the tail, or offset from the tail without removing it from the buffer
	T peek(unsigned int offset=0)
	{
		//make sure buffer large enough to contain item that's offset
		if (size() <= offset){
			return T();
		}

		auto val = buf_[tail_ + offset];

		//do not advance the tail, makes no change to buffer state
		return val;
	}

	void reset()
	{
		// std::lock_guard<std::mutex> lock(mutex_);
		head_ = tail_;
		full_ = false;
	}

	bool empty() const
	{
		//if head and tail are equal, we are empty
		return (!full_ && (head_ == tail_));
	}

	bool full() const
	{
		//If tail is ahead the head by 1, we are full
		return full_;
	}

	size_t capacity() const
	{
		return max_size_;
	}

	size_t size() const
	{
		size_t size = max_size_;

		if(!full_)
		{
			if(head_ >= tail_)
			{
				size = head_ - tail_;
			}
			else
			{
				size = max_size_ + head_ - tail_;
			}
		}

		return size;
	}

private:
	// std::mutex mutex_;
	std::unique_ptr<T[]> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_;
	bool full_ = 0;
};

#endif

    © 2022 GitHub, Inc.

    Terms
    Privacy
    Security
    Status
    Docs
    Contact GitHub
    Pricing
    API
    Training
    Blog
    About

