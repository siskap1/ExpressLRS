#ifndef UBXGPSNAVPVT_H_
#define UBXGPSNAVPVT_H_

#include <UbxGps.h>

typedef struct {
    // Type         Name           Unit     Description (scaling)
    uint8_t         hour;       // h        Hour of day, range 0..23 UTC
    uint8_t         min;        // min      Minute of hour, range 0..59 UTC
    uint8_t         sec;        // s        Seconds of minute, range 0..60 UTC
    uint8_t         fixType;    // -        GNSSfix Type, range 0..5        0=no fix, 1=deadreckoning, 2=2D, 3=3D, 4=GNSS+deadreckoning, 5=time only
    uint8_t         flags;      // -        Fix Status Flags (see graphic below) 0b76543210  7+6=carrierSoln, 5=headVehValid(sensfusiononly), 4+3+2=psmState, 1=diffSoln(diff. applied), 0=gnssFixOK (within accuracy masks)
    uint8_t         numSV;      // -        Number of satellites used in Nav Solution
    int32_t         lon;        // deg      Longitude (1e-7)
    int32_t         lat;        // deg      Latitude (1e-7)
    int32_t         height;     // mm       Height above Ellipsoid
    int32_t         hMSL;       // mm       Height above mean sea level
    uint32_t        hAcc;       // mm       Horizontal Accuracy Estimate
    uint32_t        vAcc;       // mm       Vertical Accuracy Estimate
    int32_t         gSpeed;     // mm/s     Ground Speed (2-D)
    int32_t         headMot;    // deg      Heading of motion 2-D (1e-5)
    uint32_t        sAcc;       // mm/s     Speed Accuracy Estimate
    uint32_t        headAcc;    // deg      Heading Accuracy Estimate (1e-5)
    uint16_t        pDOP;       // -        Position DOP (0.01)
    uint32_t        timestamp;  // ms       millis() timestamp of when this position was stored, so that we know it's age
} gps_sol_struct;

template <class T>
class UbxGpsNavPvt : public UbxGps<T>
{
  public:
    // Type         Name           Unit     Description (scaling)
    uint32_t        iTOW;       // ms       GPS time of week of the navigation epoch. See the description of iTOW for details
    uint16_t        year;       // y        Year UTC
    uint8_t         month;      // month    Month, range 1..12 UTC
    uint8_t         day;        // d        Day of month, range 1..31 UTC
    uint8_t         hour;       // h        Hour of day, range 0..23 UTC
    uint8_t         min;        // min      Minute of hour, range 0..59 UTC
    uint8_t         sec;        // s        Seconds of minute, range 0..60 UTC
    uint8_t         valid;      // -        Validity Flags (see graphic below)
    uint32_t        tAcc;       // ns       Time accuracy estimate UTC
    int32_t         nano;       // ns       Fraction of second, range -1e9..1e9 UTC
    uint8_t         fixType;    // -        GNSSfix Type, range 0..5
    uint8_t         flags;      // -        Fix Status Flags (see graphic below)
    uint8_t         flags2;     // -        Reserved
    uint8_t         numSV;      // -        Number of satellites used in Nav Solution
    int32_t         lon;        // deg      Longitude (1e-7)
    int32_t         lat;        // deg      Latitude (1e-7)
    int32_t         height;     // mm       Height above Ellipsoid
    int32_t         hMSL;       // mm       Height above mean sea level
    uint32_t        hAcc;       // mm       Horizontal Accuracy Estimate
    uint32_t        vAcc;       // mm       Vertical Accuracy Estimate
    int32_t         velN;       // mm/s     NED north velocity
    int32_t         velE;       // mm/s     NED east velocity
    int32_t         velD;       // mm/s     NED down velocity
    int32_t         gSpeed;     // mm/s     Ground Speed (2-D)
    int32_t         headMot;    // deg      Heading of motion 2-D (1e-5)
    uint32_t        sAcc;       // mm/s     Speed Accuracy Estimate
    uint32_t        headAcc;    // deg      Heading Accuracy Estimate (1e-5)
    uint16_t        pDOP;       // -        Position DOP (0.01)
    uint16_t        flags3;     // -        Reserved
    uint32_t        reserved1;  // -        Reserved
    int32_t         headVeh;    // deg      Heading of vehicle (2-D)
    int16_t         magDec;     // deg      Magnetic decliantion, only ADR 4.10 and newer
    uint16_t        magAcc;     // deg      Magnetic declination accuracy, only as above
    
    gps_sol_struct  solution;   //          copy of the structure above to a place that doesn't get overwritten with every message parsing...

    UbxGpsNavPvt(T &serial) : UbxGps<T>(serial){
        this->setLength(92);
        this->setClass(0x01);
        this->setID(0x07);
    }
    
    bool isGood(){
        if (
                ((this->numSV) > 4) &&                  // at least 5 sattelites
                ((this->flags & 0x01) == 0x01) &&       // valid solution fitting in the accuracy masks of the GPS
                ((this->fixType) > 1) &&                // at least 2D fix
                ((this->fixType) < 4) &&                // 
                ((this->hAcc) < 15000UL)                // at least 15m horizontal accuracy
            ){
            return true;
        } else {
            return false;
        }
    }
    bool isJustEnough(){
        if (
                ((this->numSV) > 2) &&
                ((this->flags & 0x01) == 0x01) &&       // valid solution fitting in the accuracy masks of the GPS
                ((this->fixType) > 0) &&                // any fix, incl. dead reckoning,
                ((this->fixType) < 4)                   // except for "time only"
            ){
            return true;
        } else {
            return false;
        }
    }
    void saveSolution(){
        this->solution.hour     = this->hour;
        this->solution.min      = this->min;
        this->solution.sec      = this->sec;
        this->solution.fixType  = this->fixType;
        this->solution.flags    = this->flags;
        this->solution.numSV    = this->numSV;
        this->solution.lon      = this->lon;
        this->solution.lat      = this->lat;
        this->solution.height   = this->height;
        this->solution.hMSL     = this->hMSL;
        this->solution.hAcc     = this->hAcc;
        this->solution.vAcc     = this->vAcc;
        this->solution.gSpeed   = this->gSpeed;
        this->solution.headMot  = this->headMot;
        this->solution.sAcc     = this->sAcc;
        this->solution.headAcc  = this->headAcc;
        this->solution.pDOP     = this->pDOP;
        //this->solution.timestamp
    }
};

#endif
