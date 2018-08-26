//============================================================================
// Name        : Test.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;


///// GMOCK /////
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using testing::_;
using testing::Return;

class QueueInterface{
public:
    virtual ~QueueInterface(){;}
    virtual void enqueue(int data) = 0;
    virtual int dequeue() = 0;
};

class MockQueue : public QueueInterface{
public:
    MOCK_METHOD0(dequeue, int());
    MOCK_METHOD1(enqueue, void(int data));
};

class DataHolder {
public:
    DataHolder(QueueInterface *queue): queue(queue){;}//public constructor, passing in queue instance
    void addData(int data){
        queue->enqueue(data);
    }
    int getData(){
        return queue->dequeue();
    }
protected:
    QueueInterface *queue;
};

TEST(GMockTests, CanInstantiateDataolder){
    MockQueue myMockObj;
    DataHolder dh(&myMockObj);
}

TEST(GMockTests, CanAddDate){
    MockQueue myMockObj;
    DataHolder dh(&myMockObj);

    /*create an expectation for myMockObj that will verify that enqueue method is called
     *and passed in correct parameters
     */
    EXPECT_CALL(myMockObj, enqueue(_)); // means accepts any form of argument
    dh.addData(1);
}

TEST(GMockTests, CanAddAndGetData){
    MockQueue myMockObj;
    DataHolder dh(&myMockObj);
    EXPECT_CALL(myMockObj, enqueue(_)); // means accepts any form of argument
    EXPECT_CALL(myMockObj, dequeue()).WillOnce(Return(1)); //dequeue called once, return one
    dh.addData(1);
    int data = dh.getData();
    ASSERT_EQ(data, 1);
}




//// GTEST ////
#include <gtest/gtest.h>

TEST(abcTestCase, canAssertTrue){
    // First argument is a "test case", which groups the tests
    // Second argument describes what the test is about
    // -- THESE CANNOT HAVE AN UNDERSCORE IN THEM --

    ASSERT_TRUE(true);
}
