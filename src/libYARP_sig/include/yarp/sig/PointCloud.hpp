
#ifndef YARP_SIG_POINTCLOUD_H
#define YARP_SIG_POINTCLOUD_H

#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/PointCloud_NetworkHeader.hpp>
#include <cstring>



namespace yarp {
    namespace sig {
        class FlexPointCloud;
        class PointCloud_NetworkHeader;
        template <class T> class PointCloud;
    }
}

// TBD: Copy constructor (needed by compilers such as Intel C++)


template <class T>
class yarp::sig::PointCloud: public yarp::os::Portable
{
public:
    // Usage stuff
    virtual void resize(int width, int height)
    {
        header.width = width;
        header.height = height;
        data.resize(width * height);
    }

    int wireSizeBytes()
    {
        return sizeof(header) + header.width*header.height*(sizeof(XYZ_RGBA_DATA::_xyz)+sizeof(XYZ_RGBA_DATA::rgba));
    }

    int dataSizeBytes() const
    {
        return /*sizeof(header) +*/ header.width*header.height*(sizeof(T));
    }

    // Portable interface

    PointCloud<T>();

    virtual bool read(yarp::os::ConnectionReader& connection)
    {
        yTrace();
        connection.convertTextMode();
        yarp::sig::PointCloud_NetworkHeader _header;
        bool ok = connection.expectBlock((char*)&_header, sizeof(_header));
        if (!ok) return false;

        data.resize(_header.height * _header.width);
        std::memset((void *) data.getFirst(), 0, data.size() * sizeof(T));

        header.height = _header.height;
        header.width = _header.width;
        header.isDense = _header.isDense;

        if (header.pointType == _header.pointType) //Working
        {
            yInfo("IS MATCHIIIIING BITCHESSSS\n");
            return data.read(connection);
        }

        T *tmp = data.getFirst();

        yAssert(tmp != nullptr);

        char * d;
        if(_header.pointType == PCL_POINT_XYZ_RGBA)
        {
            d = new XYZ_RGBA_DATA;
        }
        connection.expectInt(); // Code auto-generated do not remove
        connection.expectInt(); // Code auto-generated do not remove

        if ((header.pointType & PC_XY_DATA) && (_header.pointType & PC_XY_DATA))
        {
            yInfo("I contain XY\n");
            size_t offset = getOffset(header.pointType, PC_XY_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i]+offset, sizeof(XY_DATA));
        //         yDebug() << "x: " << tmp[i].x << "y: " << tmp[i].y << "z: " << tmp[i].z;
        //         yDebug() << "r: " << (u_int8_t)tmp[i].r << "g: " << (u_int8_t)tmp[i].g << "b: " << (u_int8_t)tmp[i].b << "\n";
            }

        }
        if ((header.pointType & PC_XYZ_DATA) && (_header.pointType & PC_XYZ_DATA))
        {
            yInfo("I contain XYZ\n");
            size_t offset = getOffset(header.pointType, PC_XYZ_DATA);
            yDebug()<<"Offset..."<<offset;
            yDebug()<<"sizzzzeee"<<sizeof(XYZ_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(XYZ_DATA));
        //         yDebug() << "x: " << tmp[i].x << "y: " << tmp[i].y << "z: " << tmp[i].z;
        //         yDebug() << "r: " << (u_int8_t)tmp[i].r << "g: " << (u_int8_t)tmp[i].g << "b: " << (u_int8_t)tmp[i].b << "\n";
            }


        }
        if ((header.pointType & PC_RGBA_DATA) && (_header.pointType & PC_RGBA_DATA))
        {
            yInfo("I contain RGBA\n");
            size_t offset = getOffset(header.pointType, PC_RGBA_DATA);
            yDebug()<<"Offset..."<<offset;
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(RGBA_DATA));
        //         yDebug() << "r: " << (u_int8_t)tmp[i].r << "g: " << (u_int8_t)tmp[i].g << "b: " << (u_int8_t)tmp[i].b << "\n";
            }


        }
        if ((header.pointType & PC_INTENSITY_DATA) && (_header.pointType & PC_INTENSITY_DATA))
        {
            yInfo("I contain I\n");
            size_t offset = getOffset(header.pointType, PC_INTENSITY_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(intensity));
            }

        }
        if ((header.pointType & PC_INTEREST_DATA) && (_header.pointType & PC_INTEREST_DATA))
        {
            size_t offset = getOffset(header.pointType, PC_INTEREST_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(strength));
            }

        }
        if ((header.pointType & PC_NORMAL_DATA) && (_header.pointType & PC_NORMAL_DATA))
        {
            size_t offset = getOffset(header.pointType, PC_NORMAL_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(NORMAL_DATA));
            }

        }
        if ((header.pointType & PC_RANGE_DATA) && (_header.pointType & PC_RANGE_DATA))
        {
            yInfo("I contain RANGE\n");
            size_t offset = getOffset(header.pointType, PC_RANGE_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(range));
            }

        }
       if ((header.pointType & PC_VIEWPOINT_DATA) && (_header.pointType & PC_VIEWPOINT_DATA))
        {
           yInfo("I contain VP\n");
           size_t offset = getOffset(header.pointType, PC_VIEWPOINT_DATA);
            for(uint i=0; i<data.size(); i++)
            {
                // Copy data stripping out padding bytes ( remove unused memory to optimize size for transmission over network)
                // --> if both sender and receiver are on the same machine, can I leverage on IPC to just copy stuff
                connection.expectBlock((char*) &tmp[i] + offset, sizeof(VIEWPOINT_DATA)); // ????
            }

        }

        // if someone is foolish enough to connect in text mode,
        // let them see something readable.
        connection.convertTextMode();
        return true;
    }

    virtual bool write(yarp::os::ConnectionWriter& writer)
    {
        writer.appendBlock((char*)&header, sizeof(header));
        return data.write(writer);
    }
    virtual yarp::os::Type getType()                        { return yarp::os::Type::byName("yarp/pointCloud"); }

    virtual yarp::os::ConstString toString(int precision=-1, int width=-1)
    {
        yTrace();
        //yarp::os::ConstString("ciaoooo");
        return PointCloud< T >::toString(precision, width);
    }


    // Internal conversions
    template<class X1> bool convertTo(yarp::sig::PointCloud<X1> &out);

    yarp::sig::VectorOf<T> data;

 private:
    yarp::sig::PointCloud_NetworkHeader    header;
    size_t getOffset(int type_composite, int type_basic)
    {
        size_t offset = 0;
        auto it = offsetMap.find(std::make_pair(type_composite, type_basic));
        if(it != offsetMap.end())
        {
            offset = it->second;
        }
        return offset;

    }
};


// Maybe do a Flex type in the same way as Flex Image / ImageOf is doing?
// template <class T>
// class yarp::sig::Flex: public yarp::sig::PointCloud
// {
// public:
//
//     virtual FlexPointCloud()  {};
//     // Portable interface
// //     virtual bool read(yarp::os::ConnectionReader& reader)   { return data.read(reader);};
// //     virtual bool write(yarp::os::ConnectionWriter& writer)  { return data.write(writer);};
// //     virtual yarp::os::Type getType()                        { return yarp::os::Type::byName("yarp/pointCloud"); };
//     virtual yarp::os::ConstString toString()                { yTrace(); return yarp::os::ConstString("ciaoooo");};
//
//     virtual void *convertFromType(int newType) { return NULL;};
//     virtual void *convertToType(int newType) { return NULL;};
//
// private:
//  allocate pointCloudData dynamically;
// };



namespace yarp {
    namespace sig {
        template<> yarp::sig::PointCloud<XYZ_RGBA_DATA>::PointCloud()   { PointCloud::header.pointType=PCL_POINT_XYZ_RGBA; }
        template<> yarp::sig::PointCloud<XYZ_DATA>::PointCloud()        { PointCloud::header.pointType=PCL_POINT_XYZ; }

        template<> yarp::os::ConstString yarp::sig::PointCloud <XYZ_RGBA_DATA> ::toString(int precision, int width);
        template<> yarp::os::ConstString yarp::sig::PointCloud <XYZ_DATA> ::toString(int precision, int width);
    }
}


template<>
inline int BottleTagMap <XYZ_RGBA_DATA> ()
{
    return BOTTLE_TAG_DOUBLE;
}

template<>
inline int BottleTagMap <XYZ_DATA> ()
{
    return BOTTLE_TAG_DOUBLE;
}


namespace yarp{
    namespace sig {

    template<> yarp::os::ConstString yarp::sig::PointCloud <XYZ_RGBA_DATA> ::toString(int precision, int width)
    {
        yTrace();
        yarp::os::ConstString out;
        const size_t pointsNum = this->header.width * this->header.height;

        yInfo() << "memSize is " << dataSizeBytes() << "; data on wire are " << wireSizeBytes();
        char tmp[350];
        if(width<0)
        {
            for(size_t i=0; i<pointsNum; i++)
            {
                snprintf(tmp, 350, "% .*lf\t% .*lf\t% .*lf\n", precision, data[i].x, precision, data[i].y, precision, data[i].z);
                out+=tmp;
                snprintf(tmp, 350, "%d\t%u\t%u\t%u\n", (u_int8_t)data[i].r, (u_int8_t)data[i].g, (u_int8_t)data[i].b, (u_int8_t)data[i].a);
                out+=tmp;
            }
        }
        else
        {
            for(size_t i=0; i<pointsNum; i++)
            {
                sprintf(tmp, "% *.*lf ", width, precision, data[i].x);
                out+=tmp;
            }
        }

        if(pointsNum >= 1)
            return out.substr(0, out.length()-1);
        return out;
    }

    template<> yarp::os::ConstString yarp::sig::PointCloud <XYZ_DATA> ::toString(int precision, int width)
    {
        yTrace();
        return yarp::os::ConstString("XYZ_DATA");
    }

//     template<> bool yarp::sig::PointCloud<XYZ_RGBA_DATA>::convertTo( int ciao);
//     template<> template<class T2> bool yarp::sig::PointCloud<XYZ_RGBA_DATA>::convertTo(yarp::sig::PointCloud<T2> *out);
//     template<class X1> bool  yarp::sig::PointCloud<XYZ_RGBA_DATA>::convertTo(yarp::sig::PointCloud<X1>);

    template<>
    template<class XYZ_DATA> bool  yarp::sig::PointCloud<XYZ_RGBA_DATA>::convertTo(yarp::sig::PointCloud<XYZ_DATA> &out)
    {
        yTrace() << "\n\tConverting from XYZ_RGBA_DATA to XYZ_DATA";
        return true;
    }

}
}



// template<>
// bool yarp::sig::PointCloud<XYZ_RGBA_DATA>::convertTo( int ciao)
// {
//     yTrace();
//     return true;
// }

// template<> template<>
// bool yarp::sig::PointCloud<XYZ_RGBA_DATA>::convertTo(yarp::sig::PointCloud<XYZ_DATA> *out)
// {
//     yTrace();
//     return true;
// }


#endif // YARP_SIG_POINTCLOUD_H
