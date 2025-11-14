#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <vector>
#include "cppTypes.h"

namespace quadruped
{
	class Queue
	{
	private:
		int maxSize;
		int frontIdx;
		int rearIdx;
		Vec3<double>* queueArr;

	protected:

	public:
		explicit Queue(int tmpSize);
		virtual ~Queue(void);

		void Push(Vec3<double> datavalue);
		Vec3<double> Pop(void);
		bool IsFull(void);
		bool IsEmpty(void);
		void Clear(void);
		int Size(void);
		Vec3<double> GetQueueFront(void);
		Vec3<double> GetQueueValue(int n);
	};
}

#endif // !QUEUE_HPP