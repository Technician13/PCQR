/**
 * @file queue.cpp
 * @brief 环形队列类Queue
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "queue.hpp"
#include <cstdlib>
#include <iostream>

using namespace quadruped;

/**
 * @brief Queue类的构造函数，完成所有对象成员的内存分配及数据成员的初值装载
 * @param  tmpSize          
 */
Queue::Queue(int tmpSize)
{
	maxSize = tmpSize;
	frontIdx = 0;
	rearIdx = 0;
	queueArr = new Vec3<double>[maxSize];
	memset(queueArr, 0, sizeof(Vec3<double>) * maxSize);
}

/**
 * @brief Queue类的析构函数，释放申请的内存
 */
Queue::~Queue(void)
{
	delete queueArr;
	queueArr = NULL;
}

/**
 * @brief 将参数装载进环形队列
 * @param  value  待装载的参数          
 */
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

/**
 * @brief 将环形队列中头节点的参数弹出
 * @return Vec3<double> 
 */
Vec3<double> Queue::Pop(void)
{
	Vec3<double> val = Vec3<double>::Zero();
	if (IsEmpty())
	{
		std::cout << "Queue is empty!" << std::endl;
		return val;
	}
	val = queueArr[frontIdx];
	queueArr[frontIdx] = Vec3<double>::Zero(); //ģ������ж���
	frontIdx = (frontIdx + 1) % maxSize;

	return val;
}

/**
 * @brief 判断环形队列是否已满
 * @return true 环形队列已满
 * @return false 环形队列未满
 */
bool Queue::IsFull(void)
{
	if (maxSize == -1)
	{
		std::cout << "Create queue error!" << std::endl;
		return false;
	}
	return (rearIdx + 1) % maxSize == frontIdx;
}

/**
 * @brief 判断环形队列是否已空
 * @return true 环形队列已空 
 * @return false 环形队列未空
 */
bool Queue::IsEmpty(void)
{
	if (maxSize == -1)
	{
		std::cout << "Create queue error!" << std::endl;
		return false;
	}
	return rearIdx == frontIdx;
}

/**
 * @brief 当前环形队列的长度
 * @return int 
 */
int Queue::Size(void)
{
	return (rearIdx - frontIdx + maxSize) % maxSize;
}

/**
 * @brief 读取环形队列头节点的参数
 * @return Vec3<double> 
 */
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

/**
 * @brief 读取环形队列从头节点开始，指定节点的参数
 * @param  n  指定的节点索引              
 * @return Vec3<double> 
 */
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

/**
 * @brief 清空当前环形队列
 */
void Queue::Clear(void)
{
	while(!IsEmpty())
		Pop();
}