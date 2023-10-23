#include "trajDecoder.hpp"
#include <gtest/gtest.h>

class DecodeTraj : public ::testing::Test {
  protected:
    DecodeTraj() {}
    void SetUp() override {
        auto trajectoryID = 0x123;
		auto trajectoryVersion = TRAJECTORY_INFO_RELATIVE_TO_OBJECT;
		auto trajectoryName = "some description";
		auto nameLength = strlen(trajectoryName)-1;
		auto numberOfPointsInTraj = 3;
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto points = encodeBuffer;
		auto bufferLength = sizeof(encodeBuffer);
		bool debug = false;

		auto offset = encodeTRAJMessageHeader(
			trajectoryID,
			TRAJECTORY_INFO_RELATIVE_TO_OBJECT,
			trajectoryName,
			nameLength,
			numberOfPointsInTraj,
			points,
			sizeof(encodeBuffer),
			debug);
		ASSERT_GT(offset, 0);
		points += offset;
		for (int i = 0; i < 3; i++) {
			struct timeval tv = {1,2};
			CartesianPosition pos = {1+i,2+i,3+i,4,true,true,true,true,true};
			SpeedType spd = {1,2,true,true};
			AccelerationType acc = {1,2,true,true};
			float curvature = 12.34;
			offset = encodeTRAJMessagePoint(
				&tv,
				pos,
				spd,
				acc,
				curvature,
				points,
				sizeof(encodeBuffer) - (points-encodeBuffer),
				debug);
			ASSERT_GT(offset, 0);
			points += offset;
		}
		offset = encodeTRAJMessageFooter(
			points,
			sizeof(encodeBuffer) - (points - encodeBuffer),
			debug
		);
		ASSERT_GT(offset, 0);
		points += offset;
    }
    
    public:
    char encodeBuffer[1024];
    TrajDecoder decoder;
};

TEST_F(DecodeTraj, CheckThatAllPointsAreDecod) {
    // Cast encodeBuffer into std::vector<char>
    std::vector<char> data(encodeBuffer, encodeBuffer + sizeof(encodeBuffer));
    decoder.DecodeTRAJ(data, true);
    ASSERT_EQ(decoder.getTrajHeader().trajectoryID, 0x123);
    ASSERT_EQ(decoder.getTraj()[0].pos.xCoord_m, 1);
    ASSERT_EQ(decoder.getTraj()[1].pos.xCoord_m, 2);
    ASSERT_EQ(decoder.getTraj()[2].pos.xCoord_m, 3);
}