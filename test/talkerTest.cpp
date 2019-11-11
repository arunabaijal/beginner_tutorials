#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/ModifyMessage.h"
#include "std_msgs/String.h"

TEST(TESTSuite, modifyMessage)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::ModifyMessage>(
      "modifyMessage");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::ModifyMessage srv;
  srv.request.changeString = "Test String";
  client.call(srv);
  std::string expected = srv.response.changedString;
  EXPECT_EQ(expected, srv.request.changeString);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "talkerTest");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

