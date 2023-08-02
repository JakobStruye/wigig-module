//
// Created by Jakob Struye on 31/07/23.
//

#include "covrage.h"
#include "ns3/vector.h"
#include "ns3/core-module.h"

namespace ns3 {

    Euler::Euler() : yaw(0), pitch(0), roll(0) {}
    Euler::Euler(double az, double el) : yaw(az), pitch(el), roll(0) {}
    Euler::Euler(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll) {}

    UV::UV(double u, double v) : u(u), v(v) {}

    double UV::dist(const UV &other) const {
        return std::sqrt(std::pow(this->u - other.u, 2) + std::pow(this->v - other.v, 2));
    }

    UV UV::extrap(const UV& a, const UV& b) const {
        //From this, extrapolate over the vector b-a
        return UV(this->u + (b.u - a.u), this->v + (b.v - a.v));
    }

    bool UV::isGood() const {
        return 1 - std::pow(u, 2) - std::pow(v, 2) >= 0.0;
    }

    Dims::Dims(int width, int height) : width(width), height(height) {}

    std::ostream& operator<<(std::ostream& os, const Dims& dims) {
        os << dims.width <<"," << dims.height;
        return os;
    }

    ElLoc::ElLoc(double x, double y) : x(x), y(y) {}

    CoVRage::CoVRage(std::string poseFolder, Time interval) : poseFolder(poseFolder), interval(interval) {
    }

    WeightsVector CoVRage::GetWeights() {
        Vector3D fromDir =  GetDirection(1, 0, Simulator::Now());
        Vector3D toDir =  GetDirection(1, 0, Simulator::Now() + Seconds(0.1));
        Eigen::Quaterniond from = VecToQuat(fromDir);
        Eigen::Quaterniond to = VecToQuat(toDir);
//        Eigen::Quaterniond rot = to * from.inverse();

        bool isHor;
        bool isTight = false;
        DiagType diag;
        Dims blocks(2,2);

        Trajectory t;
        auto &pts = t.allPoints;
        std::vector<Euler> pointsEul;
        double step = 0.0005;
        double targetDist = 0.0002;
        double rotLen = 0.0;

        double progress = 0.0;
        while (progress < 1) {
            Eigen::Quaterniond newPt = from.slerp(progress, to);
            pointsEul.push_back(QuatToEuler(newPt));
            pts.push_back(EulerToUv(pointsEul.back()));
            if (pts.size() > 1) {
                double dist = pts[pts.size() - 1].dist(pts[pts.size() - 2]);
                rotLen += dist;
                step *= (targetDist / dist);
            }
            progress += step;
        }

        double pathlen = 0;
        for (size_t i = 1; i < pts.size(); i++) {
            pathlen += pts[i].dist(pts[i - 1]);
        }

        UV fromUv = EulerToUv(QuatToEuler(VecToQuat(fromDir)));
        UV toUv = EulerToUv(QuatToEuler(VecToQuat(toDir)));

        double slope = std::abs(toUv.v - fromUv.v) /
                       std::abs(toUv.u - fromUv.u);

        isHor = slope < 1;
        double modSlope = isHor ? slope : 1.0/slope;
        if (abs(modSlope) < 1/3.0)
            diag = NODIAG;
        else if (abs(modSlope) < 2/3.0)
            diag = INBETWEEN;
        else
            diag = DIAG;

        double radius = GetWidthUv(dims.width / blocks.width) / 2;
        int beamCount;
        if (isTight)
            beamCount = 14;
        else if (diag == NODIAG)
            beamCount = 14;
        else if (diag == INBETWEEN)
            beamCount = 12;
        else if (diag == DIAG)
            beamCount = 13;
        else
            std::exit(1);

        while (pts.size() < (size_t) beamCount*2) {
            //Add points until we have enough in low-mobility cases
            pts.push_back(pts.back());
        }
        double baseradius = pathlen / (beamCount - 1) / 2;
        UV *keyPoint = &(pts[0]);
        bool hasCircle = false;
        for (int i = 0; i < (int) pts.size(); i++) {
            radius = baseradius;
            UV &curPoint = pts[i];
            if (i > 0 && keyPoint->dist(curPoint) < radius) {
                continue;
            }

            UV &prevPoint = pts[std::max(0, i - 1)];
            if (!hasCircle) {
                t.beamPoints.push_back(prevPoint);
            } else {
                t.midPoints.push_back(prevPoint);
                t.syncs.push_back(t.syncs.size());
            }
            keyPoint = &prevPoint;
            hasCircle = !hasCircle;
        }
        if (!hasCircle && pts.size() == 1) {
            //Why is this?
            t.beamPoints.push_back(pts[0]);
        } else if (!hasCircle && pts.size() > 1) {
            const UV &pointA = pts[pts.size() - 2];
            const UV &pointB = pts[pts.size() - 1];
            UV extrap = pointB.extrap(pointA, pointB);
            while (keyPoint->dist(extrap) < radius) {
                extrap = extrap.extrap(pointA, pointB);
            }
            extrap = extrap.extrap(pointB, pointA);
            while (!extrap.isGood()) {
                extrap = extrap.extrap(pointB, pointA);
            }
            t.beamPoints.push_back(extrap); //Move back one, got too far.
        }

        adjustPoints(t, isHor, isTight, diag);
        Dims subarrDims(2,2);
        Eigen::MatrixXi dist = configureInterleavedRect(dims, subarrDims, isHor, isTight, diag);
        dist = combineDist(dist, t.beamMapping);

        std::vector<Euler> beamPoints;
        std::vector<Euler> midPoints;
        for (auto &p : t.beamPoints) {
            beamPoints.push_back(uvToEuler(p));
        }
        for (auto &p : t.midPoints) {
            midPoints.push_back(uvToEuler(p));
        }
        VectorCplx awv = configureAwv(dist, beamPoints);
        smoothWeights(awv, dist, midPoints, t.syncs);
        WeightsVector wv;
        for (int i = 0; i < awv.rows(); i++) {
            cplx val = awv.coeff(i,0);
            wv.push_back(std::complex<float>(val));
        }
        return wv;
    }

    Pose CoVRage::GetPose(int nodeIdx, Time time) {
        if (m_poseVecs.empty()) {
            InitializePoseVecs();
        }
        if (m_poseVecs[nodeIdx].size() == 1) {
            return m_poseVecs[nodeIdx][0];
        }

        int timeIdx = std::floor(time.GetSeconds() / interval.GetSeconds());
        return m_poseVecs[nodeIdx][timeIdx];
    }

    Vector3D CoVRage::GetDirection(int fromNodeIdx, int toNodeIdx, Time time) {
        Pose fromPose = GetPose(fromNodeIdx, time);
        Pose toPose = GetPose(toNodeIdx, time);

        Euler fromDir = fromPose.second;
        Eigen::Quaterniond fromQuat = EulerToQuat(fromDir);
        Vector3D diffVec = toPose.first - fromPose.first;
        Eigen::Vector3d diffVecEigen(diffVec.x, diffVec.y, diffVec.z);
        diffVecEigen.normalize();
        Eigen::Vector3d finalVec = fromQuat * diffVecEigen;
        return Vector3D(finalVec.x(), finalVec.y(), finalVec.z());
    }

    void CoVRage::InitializePoseVecs () {
        int nodeIdx = 0;
        while (true) {
            std::string posFilename;
            std::string rotFilename;


            posFilename = std::string(poseFolder) + std::string("NodePosition") + std::to_string(nodeIdx) + std::string(".dat");
            rotFilename = std::string(poseFolder) + std::string("NodeRotation") + std::to_string(nodeIdx) + std::string(".dat");

            std::ifstream posFile;
            std::ifstream rotFile;
            posFile.open(posFilename.c_str(), std::ifstream::in);
            if (!posFile.good()) {
                break;
            }
            rotFile.open(rotFilename.c_str(), std::ifstream::in);
            if (!rotFile.good()) {
                NS_FATAL_ERROR("Position found, rotation not found");
            }
            m_poseVecs.push_back(PoseVec());
            std::string val;
            bool isFirst = true;
            Euler firstRot;
            while (true) {
                std::string line;
                Pose pose;
                std::getline(posFile, line);
                if (posFile.eof()) {
                    break;
                }
                std::istringstream stream (line);
                std::getline(stream, val, ',');
                pose.first.x = std::stod(val);
                std::getline(stream, val, ',');
                pose.first.y = std::stod(val);
                std::getline(stream, val, ',');
                pose.first.z = std::stod(val);

                std::getline(rotFile, line);
                if (!rotFile.eof()) {
                    stream = std::istringstream (line);
                    std::getline(stream, val, ',');
                    pose.second.yaw = std::stod(val);
                    std::getline(stream, val, ',');
                    pose.second.pitch = std::stod(val);
                    std::getline(stream, val, ',');
                    pose.second.roll = std::stod(val);
                } else {
                    pose.second = firstRot;
                }
                m_poseVecs[nodeIdx].push_back(pose);
                if (isFirst) {
                    firstRot = pose.second;
                    isFirst = false;
                }
            }
            nodeIdx++;

        }

    }

    Eigen::MatrixXi CoVRage::combineDist(const Eigen::MatrixXi &dist, const std::vector<int> &mapping) {
        Eigen::MatrixXi newDist = dist;
        for (int x = 0; x < newDist.rows(); x++) {
            for (int y = 0; y < newDist.cols(); y++) {
                newDist(x, y) = newDist(x, y) >= 0 ? mapping[newDist(x, y)] : -1;
            }
        }
        return newDist;
    }

    void CoVRage::adjustPoints(Trajectory &t, bool isHor, bool isTight, DiagType diag) {
        BeamMap beamMap(4, 4);
        beamMap.setConstant(-1);

        if (!isHor && !isTight) {
            if (diag == NODIAG) {
                beamMap(0, 0) = 0;
                beamMap(0, 1) = 2;
                beamMap(0, 2) = 10;
                beamMap(0, 3) = 12;
                beamMap(1, 0) = 1;
                beamMap(1, 1) = 3;
                beamMap(1, 2) = 11;
                beamMap(1, 3) = 13;
                beamMap(2, 0) = 4;
                beamMap(2, 1) = 5;
                beamMap(2, 2) = 8;
                beamMap(2, 3) = 9;
                beamMap(3, 0) = 6;
                beamMap(3, 1) = 7;
                beamMap(3, 2) = -1;
                beamMap(3, 3) = -1;
            } else if (diag == DIAG) {
                beamMap(0, 0) = 0;
                beamMap(0, 1) = 1;
                beamMap(0, 2) = 12;
                beamMap(0, 3) = -1;
                beamMap(1, 0) = 3;
                beamMap(1, 1) = 2;
                beamMap(1, 2) = 11;
                beamMap(1, 3) = 10;
                beamMap(2, 0) = 4;
                beamMap(2, 1) = 5;
                beamMap(2, 2) = 8;
                beamMap(2, 3) = 9;
                beamMap(3, 0) = -1;
                beamMap(3, 1) = 6;
                beamMap(3, 2) = 7;
                beamMap(3, 3) = -1;
            } else if (diag == INBETWEEN) {
                beamMap(0, 0) = 0;
                beamMap(0, 1) = 1;
                beamMap(0, 2) = 11;
                beamMap(0, 3) = -1;
                beamMap(1, 0) = 3;
                beamMap(1, 1) = 2;
                beamMap(1, 2) = 10;
                beamMap(1, 3) = 9;
                beamMap(2, 0) = 5;
                beamMap(2, 1) = 4;
                beamMap(2, 2) = -1;
                beamMap(2, 3) = 8;
                beamMap(3, 0) = -1;
                beamMap(3, 1) = 6;
                beamMap(3, 2) = 7;
                beamMap(3, 3) = -1;
            }
        } else if (!isHor && isTight) {

            beamMap(0, 0) = 0;
            beamMap(0, 1) = 2;
            beamMap(0, 2) = 1;
            beamMap(0, 3) = 3;
            beamMap(1, 0) = 6;
            beamMap(1, 1) = 4;
            beamMap(1, 2) = 5;
            beamMap(1, 3) = 7;
            beamMap(2, 0) = 8;
            beamMap(2, 1) = 10;
            beamMap(2, 2) = 9;
            beamMap(2, 3) = 11;
            beamMap(3, 0) = -1;
            beamMap(3, 1) = 12;
            beamMap(3, 2) = 13;
            beamMap(3, 3) = -1;
        } else if (isHor && !isTight) {
            if (diag == NODIAG) {
                beamMap(0, 0) = 0; //HOR
                beamMap(0, 1) = 2;
                beamMap(0, 2) = 4;
                beamMap(0, 3) = 6;
                beamMap(1, 0) = 1;
                beamMap(1, 1) = 3;
                beamMap(1, 2) = 5;
                beamMap(1, 3) = 7;
                beamMap(2, 0) = 10;
                beamMap(2, 1) = 12;
                beamMap(2, 2) = 8;
                beamMap(2, 3) = -1;
                beamMap(3, 0) = 11;
                beamMap(3, 1) = 13;
                beamMap(3, 2) = 9;
                beamMap(3, 3) = -1;
            } else if (diag == DIAG) {
                beamMap(0, 0) = 0;
                beamMap(0, 1) = 3;
                beamMap(0, 2) = 4;
                beamMap(0, 3) = -1;
                beamMap(1, 0) = 1;
                beamMap(1, 1) = 2;
                beamMap(1, 2) = 5;
                beamMap(1, 3) = 6;
                beamMap(2, 0) = 12;
                beamMap(2, 1) = 11;
                beamMap(2, 2) = 8;
                beamMap(2, 3) = 7;
                beamMap(3, 0) = -1;
                beamMap(3, 1) = 10;
                beamMap(3, 2) = 9;
                beamMap(3, 3) = -1;
            } else if (diag == INBETWEEN) {
                beamMap(0, 0) = 0;
                beamMap(0, 1) = 3;
                beamMap(0, 2) = 5;
                beamMap(0, 3) = -1;
                beamMap(1, 0) = 1;
                beamMap(1, 1) = 2;
                beamMap(1, 2) = 4;
                beamMap(1, 3) = 6;
                beamMap(2, 0) = 11;
                beamMap(2, 1) = 10;
                beamMap(2, 2) = -1;
                beamMap(2, 3) = 7;
                beamMap(3, 0) = -1;
                beamMap(3, 1) = 9;
                beamMap(3, 2) = 8;
                beamMap(3, 3) = -1;
            }
        }
        if (isHor && isTight) {
            beamMap(0, 0) = 0;
            beamMap(0, 1) = 6;
            beamMap(0, 2) = 8;
            beamMap(0, 3) = -1;
            beamMap(1, 0) = 2;
            beamMap(1, 1) = 4;
            beamMap(1, 2) = 10;
            beamMap(1, 3) = 12;
            beamMap(2, 0) = 1;
            beamMap(2, 1) = 5;
            beamMap(2, 2) = 9;
            beamMap(2, 3) = 13;
            beamMap(3, 0) = 3;
            beamMap(3, 1) = 7;
            beamMap(3, 2) = 11;
            beamMap(3, 3) = -1;
        }

        Eigen::Map<Eigen::RowVectorXi> beamVec(beamMap.data(), beamMap.size());
        t.beamMapping = {};
        for (int i = 0; i < beamVec.cols(); i++) {
            t.beamMapping.push_back(beamVec(i));
        }

    }

    Eigen::MatrixXi CoVRage::configureInterleavedRect(Dims dims, Dims subarrsPerBlock, bool isHor, bool isTight, DiagType diag) {
        if (! (subarrsPerBlock.width ==2  && subarrsPerBlock.height == 2)) {
            std::cout << "Bad subarr config, spacing violation" << std::endl;
        }
        Eigen::MatrixXi dist(dims.width, dims.height);
        Dims blockSize = Dims(dims.width / blocks.width, dims.height / blocks.height);

        for (int x = 0; x < dims.width; x++) {
            for (int y = 0; y < dims.height; y++) {
                int xBlockIdx = x / blockSize.width; //Rounds down
                int xSubarrIdx = x % blockSize.width % subarrsPerBlock.width;
                int yBlockIdx = y / blockSize.height; //Rounds down
                int ySubarrIdx = y % blockSize.height % subarrsPerBlock.height;
                dist(x, y) = (yBlockIdx * subarrsPerBlock.height + ySubarrIdx) +
                             ((xBlockIdx * subarrsPerBlock.width + xSubarrIdx) * subarrsPerBlock.height * blocks.height);

                if (!isTight && !isHor) {
                    if (diag == NODIAG) {
                        if (dist(x, y) % 4 == 3) {
                            if (x < 16 || x >= 48) {
                                dist(x, y) = -1;
                            } else if (dist(x, y) >= 8) {
                                dist(x, y) -= 8;
                            }

                        }
                    } else if (diag == INBETWEEN || diag == DIAG) {
                        if (dist(x, y) == 1 && y < 16) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 3 && y < 48) {
                            dist(x, y) = 1;
                        } else if (dist(x, y) == 3 && y >= 48) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 7 && x < 16) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 15 && x < 48) {
                            dist(x, y) = 7;
                        } else if (dist(x, y) == 15 && x >= 48) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 14 && y >= 48) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 12 && y >= 16) {
                            dist(x, y) = 14;
                        } else if (dist(x, y) == 12 && y < 16) {
                            dist(x, y) = -1;
                        }
                        if (diag == INBETWEEN) {
                            if (dist(x, y) == 2 && x < 16) {
                                dist(x, y) = -1;
                            } else if (dist(x, y) == 10 && x < 48) {
                                dist(x, y) = 2;
                            } else if (dist(x, y) == 10 && x >= 48) {
                                dist(x, y) = -1;
                            }                    }
                    }

                } else if (!isTight && isHor) {
                    if (diag == NODIAG) {
                        if (dist(x, y) >= 12) {
                            if (y < 16 || y >= 48) {
                                dist(x, y) = -1;
                            } else if (dist(x, y) >= 14) {
                                dist(x, y) -= 2;
                            }
                        }
                    } else if (diag == INBETWEEN || diag == DIAG) {
                        if (dist(x, y) == 4 && x < 16) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 12 && x < 48) {
                            dist(x, y) = 4;
                        } else if (dist(x, y) == 12 && x >= 48) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 13 && y < 16) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 15 && y < 48) {
                            dist(x, y) = 13;
                        } else if (dist(x, y) == 15 && y >= 48) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 11 && x >= 48) {
                            dist(x, y) = -1;
                        } else if (dist(x, y) == 3 && x >= 16) {
                            dist(x, y) = 11;
                        } else if (dist(x, y) == 3 && x < 16) {
                            dist(x, y) = -1;
                        }
                        if (diag == INBETWEEN) {
                            if (dist(x, y) == 8 && y < 16) {
                                dist(x, y) = -1;
                            } else if (dist(x, y) == 10 && y < 48) {
                                dist(x, y) = 8;
                            } else if (dist(x, y) == 10 && y >= 48) {
                                dist(x, y) = -1;
                            }
                        }
                    }
                }
                else if (isTight && isHor) {
                    if (dist(x, y) == 4 && x < 16) {
                        dist(x,y) = -1;
                    }
                    else if (dist(x, y) == 7 && x < 16) {
                        dist(x,y) = -1;
                    }
                    else if (dist(x, y) == 12 && x < 48) {
                        dist(x,y) = 4;
                    }
                    else if (dist(x, y) == 15 && x < 48) {
                        dist(x,y) = 7;
                    }
                    else if (dist(x, y) == 12 && x >= 48) {
                        dist(x,y) = -1;
                    }
                    else if (dist(x, y) == 15 && x >= 48) {
                        dist(x,y) = -1;
                    }
                }
                else if (isTight && !isHor) {
                    if (dist(x, y) == 1 && y < 16) {
                        dist(x,y) = -1;
                    }
                    else if (dist(x, y) == 13 && y < 16) {
                        dist(x,y) = -1;
                    }
                    else if (dist(x, y) == 3 && y < 48) {
                        dist(x,y) = 1;
                    }
                    else if (dist(x, y) == 15 && y < 48) {
                        dist(x,y) = 13;
                    }
                    else if (dist(x, y) == 3 && y >= 48) {
                        dist(x,y) = -1;
                    }
                    else if (dist(x, y) == 15 && y >= 48) {
                        dist(x,y) = -1;
                    }
                }
            }
        }
        return dist;
    }

    VectorCplx CoVRage::calcSteervec(Euler euler, Dims dims) {
        VectorCplx m(dims.getElCount());
        int ctr = 0;
        for (int x = 0; x < dims.width; x++) {
            for (int y = 0; y < dims.height; y++) {
//                ElLoc loc = ElLoc(x - ((dims.width-1)/2),y - ((dims.height-1)/2));
                ElLoc loc = ElLoc(x,y);
                cplx steer(0.0, calcSteervecEl(euler, loc));
                cplx val = std::pow(M_E, steer);
                m(ctr) = val;
                ctr++;
            }
        }

        //Put mean to zero, probably not strictly necessary
        m /= exp(cplx(0, m.array().arg().mean()));


        return m;
    }

    VectorCplx CoVRage::configureAwv(Eigen::MatrixXi dist, const std::vector<Euler>& eulers) {
        std::vector<VectorCplx> weightssets;
        for (size_t idx = 0; idx < eulers.size(); idx++) {
            weightssets.push_back(normalize(calcSteervec(eulers[idx], dims)));
        }
        VectorCplx awv = VectorCplx(dims.getElCount());
        awv.setZero();
        int ctr = 0;
        for (int x = 0; x < dims.width; x++) {
            for (int y = 0; y < dims.height; y++) {
                if (dist(x,y) >= 0) {
                    awv(ctr) = weightssets[dist(x, y)](ctr);
                }
                ctr++;
            }
        }
        return awv;
    }


    void CoVRage::smoothWeights(VectorCplx& awv, const Eigen::MatrixXi& dist, const std::vector<Euler>& midPoints, const std::vector<int>& syncs) {

        int subarrs = dist.maxCoeff() +1;
        for (int idx = 0; idx < subarrs-1; idx++) {
            int syncee = idx+1;
            int syncer = syncs[idx];
            if (syncer < 0) {
                continue;
            }
            const Euler& midPoint = midPoints[idx];
            VectorCplx steerVec = calcSteervec(midPoint, dims);
            MatrixXb selection1 = (dist.array()).cwiseEqual(syncee);
            Eigen::MatrixXcd subWeights1 = fillFrom(awv, selection1);
            VectorCplx steerVec1 = fillFrom(steerVec, selection1);
            cplx mod1 = (steerVec1.conjugate().transpose() * subWeights1)(0);
            double phase1 = std::arg(mod1);

            MatrixXb selection2 = (dist.array()).cwiseEqual(syncer);
            Eigen::MatrixXcd subWeights2 = fillFrom(awv, selection2);
            VectorCplx steerVec2 = fillFrom(steerVec, selection2);
            cplx mod2 = (steerVec2.conjugate().transpose() * subWeights2)(0);
            double phase2 = std::arg(mod2);

            multiplyWhere(awv, selection1, 1.0 / (std::exp(cplx(0.0, (phase1-phase2)))));
        }
    }

    double CoVRage::GetWidthUv(int elCount) {
        return std::sin(0.886 / (elCount * SPACING));
    }

    Eigen::Quaterniond CoVRage::VecToQuat(const Vector3D &vec) const {
        Vector3D normVec = vec /  vec.GetLength();

        double cos_theta = normVec.y;
        double theta = std::acos(cos_theta);
        double w = std::cos(theta / 2);
        double sin_theta_over_2 = std:: sin(theta / 2);
        double x = normVec.x * sin_theta_over_2;
        double y = normVec.y * sin_theta_over_2;
        double z = normVec.z * sin_theta_over_2;
        return Eigen::Quaterniond(w,x,y,z);
    }

    Eigen::Quaterniond CoVRage::EulerToQuat(const Euler& euler) const {
        //OK
        double cr = std::cos(euler.roll * 0.5);
        double sr = std::sin(euler.roll * 0.5);
        double cp = std::cos(euler.pitch * 0.5);
        double sp = std::sin(euler.pitch * 0.5);
        double cy = std::cos(euler.yaw * 0.5);
        double sy = std::sin(euler.yaw * 0.5);

        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;
        return Eigen::Quaterniond(w,y,x,z);
    }

    Euler CoVRage::QuatToEuler(const Eigen::Quaterniond& q) const {
        //OK
        Euler eul;
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.y() + q.x() * q.z());
        double cosr_cosp = 1 - 2 * (q.y() * q.y() + q.x() * q.x());
        eul.roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (q.w() * q.x() - q.y() * q.z()));
        double cosp = std::sqrt(1 - 2 * (q.w() * q.x() - q.y() * q.z()));
        eul.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.y() * q.x());
        double cosy_cosp = 1 - 2 * (q.x() * q.x() + q.z() * q.z());
        eul.yaw = std::atan2(siny_cosp, cosy_cosp);
        return eul;
    }


    UV CoVRage::EulerToUv(const Euler& eul) const {
        return UV(std::cos(eul.pitch)*std::sin(eul.yaw), std::sin(eul.pitch));
    }

    Euler CoVRage::uvToEuler(const UV& uv) {
        return Euler(std::atan2(uv.u, sqrt(1 - std::pow(uv.u, 2) - std::pow(uv.v, 2))), std::asin(uv.v), 0);
    }

    VectorCplx CoVRage::fillFrom(const VectorCplx& source, const MatrixXb& selection) {
        int resultSize = selection.count();
        VectorCplx result(resultSize);
        int resultIdx = 0;
        for (int xSource = 0; xSource < selection.cols(); xSource++) {
            for (int ySource = 0; ySource < selection.rows(); ySource++) {
                if (selection(xSource,ySource) == true) {
                    result(resultIdx) = source(xSource*selection.cols()+ySource);
                    resultIdx++;
                }
            }
        }
        assert(resultSize == resultIdx);
        return result;
    }

    void CoVRage::multiplyWhere(VectorCplx& v, const MatrixXb& selection, cplx multiplier) {
        for (int xSource = 0; xSource < selection.cols(); xSource++) {
            for (int ySource = 0; ySource < selection.rows(); ySource++) {
                if (selection(xSource,ySource) == true) {
                    v(xSource*selection.cols()+ySource) *= multiplier;
                }
            }
        }
    }

    VectorCplx CoVRage::normalize(const VectorCplx& vec) {
        return vec * 1 / sqrt(vec.size());
    }

    double CoVRage::calcSteervecEl(Euler euler, ElLoc elloc) {
        return M_PI * 2 * SPACING * (std::sin(euler.yaw) * elloc.x * std::cos(euler.pitch) + elloc.y * std::sin(euler.pitch));
    }
}
