#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/ModifyMessage.h"
#include "std_msgs/String.h"

TEST(TESTSuite, ModifyMessage)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::ModifyMessage>(
      "ModifyMessage");
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);

  beginner_tutorials::ModifyMessage srv;
  srv.request.changeString = "Test String";
  client.call(srv.request, srv.response);
  EXPECT_EQ("Test String", srv.response.changedString);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "talkerTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

