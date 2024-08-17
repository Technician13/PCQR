/**
 * @file queue.hpp
 * @brief 环形队列类Queue头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <vector>
#include "cppTypes.h"

namespace quadruped
{
	/**
	 * @brief 环形队列类
	 */
	class Queue
	{
	private:
		/** @brief 环形队列容量 */
		int maxSize;
		/** @brief 环形队列中头节点索引 */
		int frontIdx;
		/** @brief 环形队列中尾节点索引 */
		int rearIdx;
		/** @brief 三维向量数组成员 */
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