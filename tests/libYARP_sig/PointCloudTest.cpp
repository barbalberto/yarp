/*
 * Copyright (C) 2017 RobotCub Consortium
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


/**
 *
 * Tests for point cloud type
 *
 */

#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>
#include <yarp/sig/PointCloud.hpp>
#include <yarp/os/PortReaderBuffer.h>

#include "TestList.h"

using namespace yarp::sig;
using namespace yarp::os;

class PointCloudTest : public yarp::os::impl::UnitTest
{
public:
    virtual ConstString getName() override
    { return "PointCloudTest"; }


    virtual void runTests() override
    {
        report(0, "Checking XYZ_RGBA_DATA sending");
        BufferedPort< PointCloud<XYZ_RGBA_DATA> > outPort;
        Port inPort;
        checkTrue(outPort.open("/test/pointcloud/out"),"Opening output port");
        checkTrue(inPort.open("/test/pointcloud/in"),"Opening input port");
        checkTrue(NetworkBase::connect(outPort.getName(), inPort.getName()),"Checking connection");
        PointCloud<XYZ_RGBA_DATA>& testPC = outPort.prepare();
        int width  = 2;
        int height = 2;
        testPC.resize(width, height);

        for (int i=0; i<width*height; i++)
        {
            testPC.data[i].x = i;
            testPC.data[i].y = i + 1;
            testPC.data[i].z = i + 2;
            testPC.data[i].r = '1';
            testPC.data[i].g = '2';
            testPC.data[i].b = '3';
            testPC.data[i].a = '4';
        }

        yarp::os::Time::delay(0.3);

        outPort.write();

        PointCloud<XYZ_RGBA_DATA> inCloud;
        inPort.read(inCloud);

        yInfo()<<"Size out"<<testPC.dataSizeBytes();
        yInfo()<<"Size in"<<inCloud.dataSizeBytes();

        checkTrue(inCloud.dataSizeBytes() == testPC.dataSizeBytes(), "Checking size consistency");

        bool ok = true;
        for (int i=0; i<width*height; i++)
        {
            ok &= inCloud.data[i].x == i;
            yInfo("x %f %d\n",inCloud.data[i].x,i);
            ok &= inCloud.data[i].y == i + 1;
            yInfo("y %f %d\n", inCloud.data[i].y, i + 1);
            ok &= inCloud.data[i].z == i + 2;
            yInfo("z %f %d\n", inCloud.data[i].z, i + 2);
            ok &= inCloud.data[i].r == '1';
            yInfo("r %c 1\n", inCloud.data[i].r );
            ok &= inCloud.data[i].g == '2';
            yInfo("g %c 2\n", inCloud.data[i].g );
            ok &= inCloud.data[i].b == '3';
            yInfo("b %c 3\n", inCloud.data[i].b );
            ok &= inCloud.data[i].a == '4';
            yInfo("a %c 4\n", inCloud.data[i].a );

            yInfo("****************************\n\n");
        }

        checkTrue(ok, "Checking data validity");





        printf("Hello Point cloud test\n");
    }
};


static PointCloudTest pointCloudTest_instance;

yarp::os::impl::UnitTest& getPointCloudTest() {
    return pointCloudTest_instance;
}

