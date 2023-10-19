//
// Created by Jakob Struye on 31/07/23.
//

#ifndef WIGIG_MODULE_COVRAGE_H
#define WIGIG_MODULE_COVRAGE_H

#include "qd-propagation-engine.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"
#include "../src/Eigen/Geometry"
#include <fstream>

#define SPACING 0.25

namespace ns3 {

    struct Euler {
        double yaw;
        double roll;
        double pitch;

        Euler();
        static Euler from_deg(double az, double el);
        static Euler from_rad(double az, double el);
        Euler(double yaw, double roll, double pitch);
        friend std::ostream& operator<<(std::ostream& os, const Euler& euler);


    private:
        Euler(double az, double el);
    };

    struct UV {
        double u;
        double v;
        UV(double u, double v);
        double dist(const UV& other) const;
        UV extrap(const UV& a, const UV& b) const;
        bool isGood() const;
        bool isNearEdge() const;
        friend std::ostream& operator<<(std::ostream& os, const UV& uv);

    };

    struct Trajectory {
        std::vector<UV> beamPoints;
        std::vector<UV> midPoints;
        std::vector<UV> allPoints;

        std::vector<int> beamMapping;
        std::vector<int> syncs;
    };

    enum DiagType {
        DIAG,
        INBETWEEN,
        NODIAG
    };

    struct Dims {
        int width;
        int height;
        Dims(int width, int height);
        inline int getElCount() const {return width*height;}
        friend std::ostream& operator<<(std::ostream& os, const Dims& dims);

    };

    struct ElLoc {
        double x;
        double y;
        ElLoc(double x, double y);
    };

    enum PredictionType {
        DEVICE,
        MODEL,
        ORACLE,
        UNDEFINED
    };

    typedef std::pair<Vector3D, Euler> Pose;
    typedef std::vector<Pose> PoseVec;
    typedef std::vector<PoseVec> PoseVecs;
    typedef std::vector<Euler> Eulers;
    typedef std::complex<double> cplx;
    typedef Eigen::Matrix<cplx, Eigen::Dynamic, 1> VectorCplx;
    typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> BeamMap;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;




    class CoVRage {
    public:
        CoVRage(std::string poseFolder, Time interval, PredictionType predType);
        WeightsVector GetWeights();

        void SetOutfile(std::ofstream* outfile);

    private:
        Pose GetPose(int nodeIdx, int timeIdx);
        void InitializePoseVecs ();

        double GetWidthUv(int elCount);

        Eigen::MatrixXi combineDist(const Eigen::MatrixXi &dist, const std::vector<int> &mapping);
        void adjustPoints(Trajectory &t, bool isHor, bool isTight, DiagType diag);
        Eigen::MatrixXi configureInterleavedRect(Dims dims, Dims subarrsPerBlock, bool isHor, bool isTight, DiagType diag);
        VectorCplx calcSteervec (Euler euler, Dims dims);

        VectorCplx configureAwv(Eigen::MatrixXi dist, const std::vector<Euler>& eulers);
        void smoothWeights(VectorCplx& awv, const Eigen::MatrixXi& dist, const std::vector<Euler>& midPoints, const std::vector<int>& syncs);

        Vector3D getDirBetween(const Pose& from, const Pose& to);

        std::vector<Vector3D> GetDirectionsOracle(int fromNodeIdx, int toNodeIdx, Time timeStart, Time timeEnd);
        std::vector<Vector3D> GetDirectionsPredictModel(int fromNodeIdx, int toNodeIdx, Time timeStart, Time timeEnd);
        std::vector<Vector3D> GetDirectionsPredictDevice(int fromNodeIdx, int toNodeIdx, Time timeStart, Time timeEnd);

        std::string poseFolder;
        Time interval;
        PoseVecs m_poseVecs;
        std::vector<Eulers> m_rotPreds;

        Dims dims = Dims(64,64);
        Dims blocks = Dims(2,2);

        std::ofstream* outfile;
        PredictionType predType;

    };

    Eigen::Quaterniond VecToQuat(const Vector3D& vec);
    Eigen::Quaterniond EulerToQuat(const Euler& euler);
    Euler QuatToEuler(const Eigen::Quaterniond& q);
    UV EulerToUv(const Euler& eul);
    Euler uvToEuler(const UV& uv);
    VectorCplx fillFrom(const VectorCplx& source, const MatrixXb& selection);
    void multiplyWhere(VectorCplx& v, const MatrixXb& selection, cplx multiplier);
    VectorCplx normalize(const VectorCplx& vec);
    double calcSteervecEl(Euler euler, ElLoc elloc);

}
#endif//WIGIG_MODULE_COVRAGE_H
