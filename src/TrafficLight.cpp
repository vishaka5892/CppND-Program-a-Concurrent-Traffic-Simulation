#include <iostream>
#include <random>
#include <future>
#include <queue>

#include "TrafficLight.h"


/* Implementation of class "MessageQueue" */


template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
       
  // perform queue modification under the lock
        std::unique_lock<std::mutex> uLock(_mutex);
        _cond.wait(uLock, [this] { return !_messages.empty(); }); // pass unique lock to condition variable
        // remove last vector element from queue
        T msg = std::move(_messages.back());
        _messages.pop_back();
        return msg; // will not be copied due to return value optimization (RVO) in C++
 }

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
  
// perform vector modification under the lock
   std::lock_guard<std::mutex> uLock(_mutex);
// add vector to queue
  /*  std::cout << "   Message " << msg << " has been sent to the queue" << std::endl;
  */
    _messages.push_back(std::move(msg));
    _cond.notify_one(); // notify client after pushing new Vehicle into vector
}


/* Implementation of class "TrafficLight" */
 
TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::kRed;
   _mesg_queue = std::make_shared<MessageQueue<TrafficLightPhase>>();
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
  while(true)
  {
    std::this_thread::sleep_for( std::chrono::milliseconds(1000));
	TrafficLightPhase phase = _mesg_queue->receive();
    if(phase == TrafficLightPhase::kGreen)
      return;
  }
}

TrafficLightPhase TrafficLight::getCurrentPhase() const
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class. 
  threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
  std::chrono::time_point<std::chrono::system_clock> lastUpdate;
  std::random_device rd;
  std::uniform_int_distribution<int> distribution(4,6);
  long cycle_duration = distribution(rd);
  lastUpdate = std::chrono::system_clock::now();
  
  std::unique_lock<std::mutex> lock(_mutex);
  std::cout << "Traffic Light #" << _id << "::cycleThroughtPhases: thread id = " << std::this_thread::get_id() << std::endl;
  lock.unlock();

  // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
  
 // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 
 while (true)
 {
	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	long timeSinceUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
   
    // and toggles the current phase of the traffic light between red and green and sends an update method 
      if ( timeSinceUpdate >= cycle_duration )
        {
            if (_currentPhase == TrafficLightPhase::kRed)
                _currentPhase = TrafficLightPhase::kGreen;
            else
                _currentPhase = TrafficLightPhase::kRed;
        // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
            // Send update to the message queue and wait
            auto mesg = _currentPhase;
            auto is_sent = std::async( std::launch::async, &MessageQueue<TrafficLightPhase>::send,_mesg_queue, std::move(mesg) );
            is_sent.wait();

            // Reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();

            // Randomly choose the cycle duration for the next cycle
            cycle_duration = distribution(rd);
        }
    } 
}
