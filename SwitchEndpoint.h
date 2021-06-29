#ifndef SWITCH_ENDPOINT_H
#define SWITCH_ENDPOINT_H

extern "C"
{
    #include "zcl.h"
    #include "OnOff.h"
    #include "OOSC.h"
}

#include "Endpoint.h"
#include "BlinkTask.h"

// List of cluster instances (descriptor objects) that are included into the endpoint
struct OnOffClusterInstances
{
    tsZCL_ClusterInstance sOnOffClient;
    tsZCL_ClusterInstance sOnOffServer;
    tsZCL_ClusterInstance sOnOffConfigServer;
} __attribute__ ((aligned(4)));


class SwitchEndpoint: public Endpoint
{    
protected:
    tsZCL_EndPointDefinition sEndPoint;
    OnOffClusterInstances sClusterInstance;
    tsCLD_OnOff sOnOffClientCluster;
    tsCLD_OnOff sOnOffServerCluster;
    tsCLD_OOSC sOnOffConfigServerCluster;
    tsCLD_OnOffCustomDataStructure sOnOffServerCustomDataStructure;

    BlinkTask blinkTask;

public:
    SwitchEndpoint();
    virtual void init();

    bool getState() const;
    void switchOn();
    void switchOff();
    void toggle();
    bool runsInServerMode() const;

protected:
    void doStateChange(bool state);
    void reportState();
    void sendCommandToBoundDevices();
    void reportStateChange();

protected:
    virtual void registerServerCluster();
    virtual void registerClientCluster();
    virtual void registerOnOffConfigServerCluster();
    virtual void registerEndpoint();
    virtual void handleClusterUpdate(tsZCL_CallBackEvent *psEvent);
};

#endif // SWITCH_ENDPOINT_H
