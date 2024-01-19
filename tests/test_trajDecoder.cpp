#include "trajDecoder.hpp"
#include <gtest/gtest.h>

class DecodeTraj : public ::testing::Test {
  protected:
    DecodeTraj() {}
    void SetUp() override {
		auto trajectoryVersion = TRAJECTORY_INFO_RELATIVE_TO_OBJECT;
		auto trajectoryName = "some description";
		auto nameLength = strlen(trajectoryName)-1;
		auto numberOfPointsInTraj = 3;
		memset(encodeBuffer, 0, sizeof(encodeBuffer));
		auto points = encodeBuffer;
		auto bufferLength = sizeof(encodeBuffer);
		bool debug = false;

		MessageHeaderType header = {0};
		auto offset = encodeTRAJMessageHeader(
			&header,
			trajectoryID,
			TRAJECTORY_INFO_RELATIVE_TO_OBJECT,
			trajectoryName,
			nameLength,
			nTrajPoints,
			points,
			sizeof(encodeBuffer),
			debug);
		ASSERT_GT(offset, 0);
		points += offset;
		for (double i = 0; i < nTrajPoints; i++) { // Using float as loop counter to avoid ugly explicit casts
			struct timeval tv = {1,2};
			CartesianPosition pos = {1+i,2+i,3+i,4,true,true,true,true,true};
			SpeedType spd = {1+i,2+i,true,true};
			AccelerationType acc = {1+i*2,2+i*2,true,true};
			float curvature = 12.34+i;
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
    
    char encodeBuffer[1024];
	int const nTrajPoints = 10;
	int trajectoryID = 666;
    TrajDecoder decoder;
};

TEST_F(DecodeTraj, CheckThatAllPointsAreDecoded) {
    std::vector<char> data(encodeBuffer, encodeBuffer + sizeof(encodeBuffer));
    decoder.DecodeTRAJ(data, true);
    ASSERT_EQ(decoder.getTrajHeader().trajectoryID, 666);
	for (int i=0; i < nTrajPoints; i++){
		ASSERT_EQ(decoder.getTraj()[i].pos.xCoord_m, 1+i);
		ASSERT_EQ(decoder.getTraj()[i].pos.yCoord_m, 2+i);
		ASSERT_EQ(decoder.getTraj()[i].pos.zCoord_m, 3+i);
		ASSERT_EQ(decoder.getTraj()[i].pos.isXcoordValid, true);
		ASSERT_EQ(decoder.getTraj()[i].pos.isYcoordValid, true);
		ASSERT_EQ(decoder.getTraj()[i].pos.isZcoordValid, true);

		ASSERT_EQ(decoder.getTraj()[i].spd.longitudinal_m_s, 1+i);
		ASSERT_EQ(decoder.getTraj()[i].spd.lateral_m_s, 2+i);
		ASSERT_EQ(decoder.getTraj()[i].spd.isLateralValid, true);
		ASSERT_EQ(decoder.getTraj()[i].spd.isLongitudinalValid, true);

		ASSERT_EQ(decoder.getTraj()[i].acc.longitudinal_m_s2, 1+i*2);
		ASSERT_EQ(decoder.getTraj()[i].acc.lateral_m_s2, 2+i*2);
		ASSERT_EQ(decoder.getTraj()[i].acc.isLateralValid, true);
		ASSERT_EQ(decoder.getTraj()[i].acc.isLongitudinalValid, true);

		EXPECT_NEAR(decoder.getTraj()[i].curvature, 12.34+i, 0.0001);
	}
}