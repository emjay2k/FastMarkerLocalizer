/**
 * queuebuffer.h
 *
 *	Implements a queuebuffer
 *
 *  Created on: 03.04.2012
 *      Author: jung
 */

#ifndef QUEUEBUFFER_H_
#define QUEUEBUFFER_H_

#include <list>
#include "generic/types.h"

using namespace std;

namespace fml {

template <class T>
class QueueBuffer {

protected:
	static const size_t kDefaultSize = 10;
	size_t max_size_;
	list<T> data_;	// the backing data structure

public:
	QueueBuffer() {
		max_size_ = kDefaultSize;
		list<T> buffer;	// the buffer
	}

	QueueBuffer(const size_t size) {
		max_size_ = size;
		data_ = list<T>();
	}

	virtual ~QueueBuffer() {
		free();
	}

	void enqueue(const T& element) {
		if(isFull())
			data_.pop_back();
		data_.push_front(element);
	}

	const T dequeue() {
		T t = data_.back();
		data_.pop_back();
		return(t);
	}

	const T front() const {
		return(data_.front());
	}

	const T back() const {
		return(data_.back());
	}

	list<T>* get_data() {
		return(&data_);
	}

	inline size_t get_size() const {
		return(data_.size());
	}

	inline bool isEmpty() const {
		return(get_size() == 0);
	}

	inline bool isFull() const {
		return(get_size() >= max_size_);
	}

	void free() {
		data_.clear();
	}

	void resize(const size_t size) {
		int num_delete = this->max_size_ - size;
		for(size_t i = 0; i < num_delete; ++i)
			data_.pop_back();
		max_size_ = size;
	}
};

} /* namespace fml */

#endif /* QUEUEBUFFER_H_ */
