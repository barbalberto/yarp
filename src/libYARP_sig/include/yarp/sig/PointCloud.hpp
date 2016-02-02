
#ifndef YARP_SIG_POINTCLOUD_H
#define YARP_SIG_POINTCLOUD_H

#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/PointCloudTypes.hpp>
#include <yarp/sig/PointCloud_NetworkHeader.hpp>



namespace yarp {
    namespace sig {
        class PointCloud;
        class PointCloud2Base;
        template <class T> class PointCloud2;
        template<> bool VectorOf <XYZ_RGBA_DATA> ::read(yarp::os::ConnectionReader& connection);
        template<> bool VectorOf <XYZ_RGBA_DATA> ::write(yarp::os::ConnectionWriter& connection);

//         template<> yarp::os::ConstString yarp::sig::PointCloud2 <XYZ_RGBA_DATA> ::toString();
//         template<> bool PointCloud2 <XYZ_RGBA_DATA> ::toString();

        //         template<> bool VectorOf <XYZ_RGBA_DATA> ::write(yarp::os::ConnectionWriter& connection);
        class PointCloud_NetworkHeader;
    }
}


YARP_BEGIN_PACK
class VectorPortContentHeader
{
public:
    yarp::os::NetInt32 listTag;
    yarp::os::NetInt32 listLen;
};
YARP_END_PACK

class yarp::sig::PointCloud : public yarp::os::Portable
{
private:
    int pointType;
    VectorOf<int> rgba;
    VectorOf<RGBA_DATA> xyzColor;


public:
    PointCloud();
    virtual ~PointCloud();

    // Portable interface
    virtual bool read(yarp::os::ConnectionReader& reader);
    virtual bool write(yarp::os::ConnectionWriter& writer);
    virtual yarp::os::Type getType();

    // Yarp PointCloud stuff
    virtual bool setPointType(int type);
    virtual bool getPointType(int *type);

private:
    yarp::sig::PointCloud_NetworkHeader    header;

};
template <class T>
class yarp::sig::PointCloud2/*Base*/: public yarp::os::Portable
{
public:
    // Portable interface
    virtual bool read(yarp::os::ConnectionReader& reader)   { return data.read(reader);};
    virtual bool write(yarp::os::ConnectionWriter& writer)  { return data.write(writer);};
    virtual yarp::os::Type getType()                        { return yarp::os::Type::byName("yarp/pointCloud"); };
    virtual yarp::os::ConstString toString()                { yTrace(); return yarp::os::ConstString("ciaoooo");};

    yarp::sig::VectorOf<T> data;
//     void  *data;
// protected:
//     virtual bool setImplementation(void *_p) {data = _p;};

private:
    yarp::sig::PointCloud_NetworkHeader    header;
};


// Maybe do it in the same way as ImageOf / Image is doing?
// template <class T>
// class yarp::sig::PointCloud2: public yarp::sig::PointCloud2Base
// {
// public:
//
//     virtual PointCloud2()  { PointCloud2Base::setImplementation(&data);};
//     // Portable interface
// //     virtual bool read(yarp::os::ConnectionReader& reader)   { return data.read(reader);};
// //     virtual bool write(yarp::os::ConnectionWriter& writer)  { return data.write(writer);};
// //     virtual yarp::os::Type getType()                        { return yarp::os::Type::byName("yarp/pointCloud"); };
//     virtual yarp::os::ConstString toString()                { yTrace(); return yarp::os::ConstString("ciaoooo");};
//
//     virtual void *convertFromType(int newType) { return NULL;};
//     virtual void *convertToType(int newType) { return NULL;};
//     yarp::sig::VectorOf<T> data;
//
// private:
//     yarp::sig::PointCloud_NetworkHeader    header;
// };


template<>
inline int BottleTagMap <XYZ_RGBA_DATA> ()
{
    return BOTTLE_TAG_DOUBLE;
}


template<>
bool yarp::sig::VectorOf <XYZ_RGBA_DATA> ::read(yarp::os::ConnectionReader& connection)
{
    yTrace();
    return true;
}

template<>
bool yarp::sig::VectorOf <XYZ_RGBA_DATA> ::write(yarp::os::ConnectionWriter& connection)
{
    yTrace();
    VectorPortContentHeader header;  // maybe not needed, since the PC will have an header on his own

    header.listTag = (BOTTLE_TAG_LIST | getBottleTag());
    header.listLen = (int)getListSize();

    connection.appendBlock((char*)&header, sizeof(header));
    const char *ptr = getMemoryBlock();
    int elemSize=getElementSize();
    yAssert(ptr != NULL);

    connection.appendExternalBlock(ptr, elemSize*getListSize());

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();

    return !connection.isError();
}


namespace yarp{
    namespace sig {

template<>
yarp::os::ConstString yarp::sig::PointCloud2 <XYZ_RGBA_DATA> ::toString()
{
    yTrace();
    return yarp::os::ConstString("XYZ_RGBA_DATA");
}
    }}

#endif // YARP_SIG_POINTCLOUD_H
