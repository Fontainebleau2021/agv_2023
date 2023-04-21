// condition_variable::notify_one
#include <iostream>           // std::cout
#include <thread>             // std::thread
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

std::mutex mtx;
std::condition_variable produce,consume;

int cargo = 0;     // shared value by producers and consumers

void consumer () {
  std::unique_lock<std::mutex> lck(mtx);
  while (cargo==0) consume.wait(lck);
  std::cout << cargo << '\n';
  cargo-=1;
  produce.notify_one();
}

void producer (int id) {
  std::unique_lock<std::mutex> lck(mtx);
  while (cargo>100) produce.wait(lck);
  cargo += 1;
  consume.notify_one();
}

int main ()
{
  std::thread consumers[100],producers[100];
  // spawn 10 consumers and 10 producers:
  for (int i=0; i<100; ++i) {
    consumers[i] = std::thread(consumer);
    producers[i] = std::thread(producer,i+1);
  }

  // join them back:
  for (int i=0; i<100; ++i) {
    producers[i].join();
    consumers[i].join();
  }

  return 0;
}