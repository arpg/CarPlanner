/*! \file
* \brief Main include.
*
* This is the only file you have to include in order to use the 
* complete threadpool library.
*
* Copyright (c) 2005-2007 Philipp Henkel
*
* Use, modification, and distribution are  subject to the
* Boost Software License, Version 1.0. (See accompanying  file
* LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*
* http://threadpool.sourceforge.net
*
*/

#ifndef THREADPOOL_HPP_INCLUDED
#define THREADPOOL_HPP_INCLUDED

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread/condition.hpp>


//namespace boost {
//namespace threadpool
//{

//class pool;

//// our worker thread objects
//class Worker {
//public:
//    Worker(pool &s) : m_pool(s) { }
//    inline void operator()();
//private:
//    pool &m_pool;
//};

//class Handler {
//public:
//    template<class F>
//    Handler(pool &s,const F& functor) : m_pool(s), m_f(functor) { }
//    inline void operator()();
//private:
//    const function0<void>& m_f;
//    pool& m_pool;
//};

//// the actual thread pool
//class pool {
//public:
//    inline pool(size_t = 0);
//    template<class F>
//    inline void schedule(const F& f);
//    inline pool& size_controller(){ return *this; }
//    inline void resize(const size_t& newSize );
//    inline void wait();
//    inline void IncrementTaskCount()
//    {
//        {
//            boost::mutex::scoped_lock lock(m_CounterMutex);
//            m_nCounter++;
//            std::cout << "Adding a task. Current task count is " << m_nCounter  << std::endl;
//            fflush(stdout);
//        }
//        m_CounterCondition.notify_all();
//    }

//    inline void DecrementTaskCount()
//    {
//        {
//            boost::mutex::scoped_lock lock(m_CounterMutex);
//            m_nCounter--;
//            std::cout << "Finished a task. Current task count is " << m_nCounter  << std::endl;
//            fflush(stdout);
//        }
//        m_CounterCondition.notify_all();
//    }

//    inline ~pool();
//private:
//    boost::mutex m_CounterMutex;
//    boost::condition m_CounterCondition;
//    int m_nCounter;
//    // need to keep track of threads so we can join them
//    std::vector< std::unique_ptr<boost::thread> > workers;

//    // the io_service we are wrapping
//    boost::asio::io_service service;
//    boost::asio::io_service::work working;
//    friend class Worker;
//};

//// all the workers do is execute the io_service
//void Worker::operator()() { m_pool.service.run(); }

//void Handler::operator ()()
//{
//    try
//    {
//        m_f();
//    }catch(...){
//        std::cout << "Exception caught when running thread pool" << std::endl;
//    }
//    m_pool.DecrementTaskCount();
//};

//// the constructor just launches some amount of workers
//pool::pool(size_t threads) : working(service), m_nCounter(0)
//{
//    for(size_t i = 0;i<threads;++i){
//        workers.push_back(
//                    std::unique_ptr<boost::thread>(
//                        new boost::thread(Worker(*this))
//                        )
//                    );
//    }
//}

//void pool::resize(const size_t& newSize )
//{
//    for(size_t i = 0;i<newSize;++i){
//        if(workers.size() <= i){
//            workers.push_back(
//                        std::unique_ptr<boost::thread>(
//                            new boost::thread(Worker(*this))
//                            ));
//        }
//    }
//}

//// add new work item to the pool
//template<typename F>
//void pool::schedule(const F& functor)
//{
//    //service.post(functor);
//    IncrementTaskCount();
//    std::shared_ptr<Handler> handlerPtr = std::make_shared<Handler>(*this,functor);
//    service.post(boost::bind(&Handler::operator(),*handlerPtr));
//}

//// the destructor joins all threads
//void pool::wait()
//{
//    //try to lock the counter
//    boost::mutex::scoped_lock lock(m_CounterMutex);

//    while(m_nCounter != 0){
//        //give up the lock and wait until someone signals this
//        m_CounterCondition.wait(lock);
//    }
//}

//pool::~pool(){
//    service.stop();
//    for(size_t i = 0;i<workers.size();++i){
//        workers[i]->join();
//    }
//}

//}
//}


#include "./threadpool/future.hpp"
#include "./threadpool/pool.hpp"

#include "./threadpool/pool_adaptors.hpp"
#include "./threadpool/task_adaptors.hpp"


#endif // THREADPOOL_HPP_INCLUDED

