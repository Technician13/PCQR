#include "queue.hpp"
#include <cstdlib>
#include <iostream>

using namespace quadruped;


Queue::Queue(int tmpSize)
{
	maxSize = tmpSize;
	frontIdx = 0;
	rearIdx = 0;
	queueArr = new Vec3<double>[maxSize];
	memset(queueArr, 0, sizeof(Vec3<double>) * maxSize);
}


Queue::~Queue(void)
{
	delete queueArr;
	queueArr = NULL;
}


void Queue::Push(Vec3<double> value)
{
	if (IsFull())
	{
		std::cout << "Queue is full!" << std::endl;
		return;
	}
	queueArr[rearIdx] = value;
	rearIdx = (rearIdx + 1) % maxSize;
}


Vec3<double> Queue::Pop(void)
{
	Vec3<double> val = Vec3<double>::Zero();
	if (IsEmpty())
	{
		std::cout << "Queue is empty!" << std::endl;
		return val;
	}
	val = queueArr[frontIdx];
	queueArr[frontIdx] = Vec3<double>::Zero(); 
	frontIdx = (frontIdx + 1) % maxSize;

	return val;
}


bool Queue::IsFull(void)
{
	if (maxSize == -1)
	{
		std::cout << "Create queue error!" << std::endl;
		return false;
	}
	return (rearIdx + 1) % maxSize == frontIdx;
}


bool Queue::IsEmpty(void)
{
	if (maxSize == -1)
	{
		std::cout << "Create queue error!" << std::endl;
		return false;
	}
	return rearIdx == frontIdx;
}


int Queue::Size(void)
{
	return (rearIdx - frontIdx + maxSize) % maxSize;
}


Vec3<double> Queue::GetQueueFront(void)
{
	if (IsEmpty())
	{
		std::cout << "queue empty!" << std::endl;
		return Vec3<double>::Zero();
	}
	else
		return queueArr[frontIdx];
}


Vec3<double> Queue::GetQueueValue(int n)
{
	if (IsEmpty())
	{
		std::cout << "queue empty!" << std::endl;
		return Vec3<double>::Zero();
	}

	else
	{
		if (n >= Size())
			return queueArr[(frontIdx + Size()) % maxSize];
		else
			return queueArr[(frontIdx + n) % maxSize];
	}	
}


void Queue::Clear(void)
{
	while(!IsEmpty())
		Pop();
}