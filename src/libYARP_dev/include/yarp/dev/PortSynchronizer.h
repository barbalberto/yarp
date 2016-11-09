/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Alberto Cardellino
 * email:   alberto.cardellino@iit.it
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef YARP_DEV_PORT_SYNCHRONIZER
#define YARP_DEV_PORT_SYNCHRONIZER

#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/DummyConnector.h>

namespace yarp {
    namespace dev {
        class ISynch;
        class PortSynchronizer;
        typedef enum
        {
            NO_SYNCH                    = 0,
            CUSTOM                      = 1,
            PROGRESSIVE_NUMBER          = 2,
            TIMESTAMP                   = 3
        } SynchPolicy;
    }
}

/**
 * @ingroup dev_iface_other
 *
 * ISynch is an interface meant to specify a custom function to synchronize
 * data coming from multiple ports.
 */


class YARP_dev_API yarp::dev::ISynch
{
public:
    virtual bool read(yarp::os::ConnectionReader& reader) = 0;
    virtual bool synch(yarp::sig::VectorOf<yarp::os::Portable *> &data) = 0;
    bool virtual getData(yarp::sig::VectorOf<yarp::os::PortReader *> &data) = 0;
};


/**
 * @ingroup dev_iface_other
 *
 * PortSynchronizer is an class providing synchronization functionality for
 * data coming from multiple ports. The data are read by this class through
 * the ports provided and then bufferized and synchronized using the specified
 * policy.
 *
 * Synchronized data can the be retrieved by the 'getData' method.
 */

class MyReader : public yarp::os::PortReader
{
    int readerId;
    int lastIndex;   // Used to handle the buffer as a circular buffer
    double startTime;

    yarp::os::Port *myPort;
    std::vector<yarp::os::DummyConnector> buffer;

public:

    bool initted;
    MyReader() : readerId(0), lastIndex(0), initted(false), startTime(0) {};

    bool config(int pluto, yarp::os::Port *port, int bufferDimension = 10)
    {
        initted = true;
        myPort = port;
        readerId = pluto;
        startTime = yarp::os::Time::now();
        buffer.resize(bufferDimension);
    };

    virtual bool read(yarp::os::ConnectionReader& reader)
    {
        yTrace();
        yInfo() << "Id: " << readerId;
        yInfo() << "initted: " << initted;
        yInfo() << "Type is " << getReadType().getName();
        yInfo() << "Pre Size is " << reader.getSize();

//         yarp::os::Bottle b;
//         b.read(reader);
//         b.isNull() ? yError() << "Null Bottle  - Id: " << pippo : yInfo() << "Valid Bottle - Id: " << pippo;

//         yarp::os::DummyConnector dummy;
//         yarp::os::ConnectionWriter &writer = dummy.getCleanWriter();

//         b.write(writer);

//         reader.readFromStream();
//         writer.wr
//         object.write(dummy.getWriter()); // reset the connection and then write to it
//         PortableObject newObject;
//         newObject.read(dummy.getReader()); // write from the connection to the new object

        /* Suggerimento si Alì: Fatto
         *
         * Creare dummy connection reader
         * Leggere dal connection reader vero e scrivere nel dummy ConnectionReader
         * Quando l'utente vuole i dati, farglieli leggere dal dummy ConnectionReader.
         * Piuttosto, vettore di ConnectionReaders??
         *
         * */

        int size = reader.getSize();
        char *data = new char[size];  // right now this requires a double memcpy... can this be optimized somehow??
        reader.expectBlock(data, size);

//         yInfo() << "Post: Size is " << dummy.getReader().getSize();

//         char *data = new char[dummy.getReader().getSize()];
//         dummy.getReader().expectBlock(data, dummy.getReader().getSize());

        lastIndex = lastIndex++ % buffer.size();
        buffer[lastIndex].getCleanWriter().appendBlock(data, size);

        yarp::os::Stamp envelop;
        myPort->getEnvelope(envelop);
        yInfo() << "Envelope is " << envelop.getCount() << " at time " << envelop.getTime() - startTime;





        return true;
    };


    bool getData(PortReader& reader)
    {
        yInfo() << "Reading some stuff";
        return reader.read(buffer[lastIndex].getReader());
    };
};

class YARP_dev_API yarp::dev::PortSynchronizer
{
private:
    int readers;
    std::vector<MyReader *> myReaders;

public:
        PortSynchronizer() : readers(0) { myReaders.clear();};
//         virtual bool read(yarp::os::ConnectionReader& reader) { yTrace(); return true;};

public:

    /**
     * Add a buffered port to the list of objects to be synchronized.
     *
     * @param port : input buffered port.
     *      The port need to be already created and existing.
     *      Life cycle of the port is outside of the scope of this object.
     *
     * @return progressive index of the vector returned from getData function.
     * TODO: describe it better
     */

    int addPort(yarp::os::Port  &port)
    {
        yTrace();
        myReaders.push_back(new MyReader());
        myReaders[readers]->config(readers, &port);
        port.setReader(*(myReaders[readers]));
        readers++;
        return 0;
    };


    bool setPolicy(yarp::dev::SynchPolicy, yarp::os::Property params);
//     bool setPolicy(yarp::dev::ISynch &policy);
    bool getData(yarp::sig::VectorOf<yarp::os::PortReader *> &data)
    {
        yDebug() << "Vector ---------------------------";
        for(int i=0; i<myReaders.size(); i++)
        {
            if(myReaders[i]->initted)
                myReaders[i]->getData(*(data[i]));
        }
        yDebug() << "Vector ||||||||||||||||||||||||||||";
    };

    bool getData(yarp::os::PortReader& reader, int index=0)
    {
        yDebug() << "---------------------------";
//         yarp::os::Bottle b;
//         for(int i=0; i<myReaders.size(); i++)
//         {
            myReaders[index]->getData(reader);
//             yInfo() << b.toString();
//         }
        yDebug() << "||||||||||||||||||||||||||||";
    };

    /* Start to read and synch data */
    bool start();

    /* Stop synching functionality */
    bool stop();

    /* Cleanup the object and stop reading from ports.
     * To be called before closing the ports! */
    bool close();
};



#endif  // YARP_DEV_PORT_SYNCHRONIZER