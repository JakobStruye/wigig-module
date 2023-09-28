//
// Created by Jakob Struye on 31/07/23.
//

#include "covrage.h"
#include "ns3/vector.h"
#include "ns3/core-module.h"

namespace ns3 {

    Euler::Euler() : yaw(0), roll(0), pitch(0) {}
    Euler::Euler(double az, double el) : yaw(az), roll(0), pitch(el) {}
    Euler::Euler(double yaw, double roll, double pitch) : yaw(yaw), roll(roll), pitch(pitch) {}

    std::ostream& operator<<(std::ostream& os, const Euler& euler) {
        os << euler.yaw <<"," << euler.roll << "," << euler.pitch;
        return os;
    }

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

    bool UV::isNearEdge() const {
        return 1 - std::pow(u, 2) - std::pow(v, 2) <= 0.00001;
    }

    std::ostream& operator<<(std::ostream& os, const UV& uv) {
        os << uv.u <<"," << uv.v;
        return os;
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
        std::vector<Vector3D> dirs = GetDirections(1, 0, Simulator::Now() - Seconds(0.05) , Simulator::Now() + Seconds(0.15));
        Vector3D& fromDir = dirs.front();
        Vector3D& toDir = dirs.back();

        bool isHor;
        bool isTight = false;
        DiagType diag;
        Dims blocks(2,2);

        Trajectory t;
        auto &pts = t.allPoints;
        std::vector<Euler> pointsEul;
        double step = 0.000005;
        double targetDist = 0.000002;
        double rotLen = 0.0;

        double progress = 0.0;
        while (progress < 1) {
            int idx = std::floor(progress * (dirs.size()-1));

            Eigen::Quaterniond thisFrom = VecToQuat(dirs[idx]);
            Eigen::Quaterniond thisTo = VecToQuat(dirs[idx+1]);
            double thisProgress = (progress * (dirs.size()-1)) - idx;

            Eigen::Quaterniond newPt = thisFrom.slerp(thisProgress, thisTo);
            pointsEul.push_back(QuatToEuler(newPt));
            pts.push_back(EulerToUv(pointsEul.back()));
            if (pts.size() > 1 && !pts[pts.size()-1].isNearEdge()&& !pts[pts.size()-2].isNearEdge()) {
                double dist = pts[pts.size() - 1].dist(pts[pts.size() - 2]);
                rotLen += dist;
                step *= (targetDist / dist);
            }
            progress += step;
        }

        double pathlen = 0;
        for (size_t i = 1; i < pts.size(); i++) {
            pathlen += pts[i].dist(pts[i - 1]);
//            std::cout << "DEBUG " << i << " DIST " <<  pts[i].dist(pts[i - 1]) << " VS " << (pointsQ[i-1]->angularDistance(*pointsQ[i])) << std::endl;
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


//        std::cout << pts.size() << "VS" << beamCount << std::endl;
        bool fewPoints = false;
        while (pts.size() < (size_t) beamCount*2) {


            //Add points until we have enough in low-mobility cases
            pts.push_back(pts.back());

            fewPoints = true;
        }
//        std::cout << pts.size() << "VS" << beamCount << std::endl;
        double baseradius = pathlen / (beamCount - 1) / 2;
        if (baseradius < 10e-10) {
            baseradius = 0;
        }
        bool hasCircle = false;
        double runningDist = 0;
        UV* circlePoint = &(pts[0]);
//        std::cout << "DEBUG" << " BASERAD " << baseradius << " PATHLEN " << pathlen <<  std::endl;
        for (int i = 0; i < (int) pts.size(); i++) {
            UV &curPoint = pts[i];
            UV &prevPoint = pts[std::max(0, i - 1)];
            runningDist += curPoint.dist(prevPoint);
            if (!fewPoints && i > 0 && runningDist < baseradius) {
                continue;
            }
//            std::cout << "DEBUG" << " ADDING " << (!hasCircle ? " BEAM " : " MIDPT ") << " AT i " << i << " RUNDIST " << runningDist << std::endl;

            if (!hasCircle) {
                t.beamPoints.push_back(prevPoint);
                circlePoint = &prevPoint;
            } else {
                t.midPoints.push_back(prevPoint);
                t.syncs.push_back(t.syncs.size());
            }
            hasCircle = !hasCircle;
            runningDist = curPoint.dist(prevPoint);
//            std::cout << "DEBUG RUNDIST REVERT " << runningDist << std::endl;
        }
        if (!hasCircle && pts.size() == 1) {
            //Why is this?
            t.beamPoints.push_back(pts[0]);
        } else if (!hasCircle && pts.size() > 1) {
            const UV &pointA = pts[pts.size() - 2];
            const UV &pointB = pts[pts.size() - 1];

            UV extrap = pointB.extrap(pointA, pointB);
            while (circlePoint->dist(extrap) < baseradius) {
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
//            std::cout << "UVEUL" << p << " " << beamPoints[beamPoints.size()-1] << std::endl;
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
        (*outfile) << std::endl << "BEAMPTS" << std::endl;
        for (auto &p : t.beamPoints) {
            (*outfile) << p << ",";
        }
        (*outfile) << std::endl;
        (*outfile) << "WEIGHTS" << std::endl;
        for (cplx c: wv) {
            (*outfile) << std::abs(c) << "," << std::arg(c) << ",";
        }
        (*outfile) << std::endl;
        return wv;
    }

    Pose CoVRage::GetPose(int nodeIdx, int timeIdx) {
        if (m_poseVecs.empty()) {
            InitializePoseVecs();
        }
        if (m_poseVecs[nodeIdx].size() == 1) {
            return m_poseVecs[nodeIdx][0];
        }
        if (m_poseVecs[nodeIdx].size() <= (unsigned) timeIdx) {
            throw std::runtime_error("Simulation running for longer than the raytracer!");
        }
        return m_poseVecs[nodeIdx][timeIdx];
    }

    std::vector<Vector3D> CoVRage::GetDirections(int fromNodeIdx, int toNodeIdx, Time timeStart, Time timeEnd) {

        int timeIdxStart = std::floor(timeStart.GetSeconds() / interval.GetSeconds());
        timeIdxStart = std::max(0, timeIdxStart);
        int timeIdxStop = std::floor(timeEnd.GetSeconds() / interval.GetSeconds());

        std::vector<Vector3D> dirs;
        for (int timeIdx = timeIdxStart; timeIdx <= timeIdxStop; timeIdx++) {
            Pose fromPose = GetPose(fromNodeIdx, timeIdx);
            Pose toPose = GetPose(toNodeIdx, timeIdx);

            Euler fromDir = fromPose.second;
            Eigen::Quaterniond fromQuat = EulerToQuat(fromDir);

            Vector3D diffVec = toPose.first - fromPose.first;
            Eigen::Vector3d diffVecEigen(diffVec.x, diffVec.y, diffVec.z);
            diffVecEigen.normalize();
            Eigen::Vector3d finalVec = fromQuat.inverse() * diffVecEigen;
            dirs.push_back(Vector3D(finalVec.x(), finalVec.y(), finalVec.z()));
        }
        return dirs;
    }

    void CoVRage::SetOutfile(std::ofstream* outfile) {
        this->outfile = outfile;
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
            Vector3D firstPos;

            while (true) {
                std::string posLine;
                std::string rotLine;
                Pose pose;
                std::getline(posFile, posLine);
                std::getline(rotFile, rotLine);
                if (posFile.eof() && rotFile.eof()) {
                    break;
                }
                std::istringstream stream;
                if (!posFile.eof() || posLine.size() > 4) {
                    stream = std::istringstream(posLine);
                    std::getline(stream, val, ',');
                    pose.first.x = std::stod(val);
                    std::getline(stream, val, ',');
                    pose.first.y = std::stod(val);
                    std::getline(stream, val, ',');
                    pose.first.z = std::stod(val);
                } else {
                    pose.first = firstPos;
                }

                if (!rotFile.eof() || rotLine.size() > 4) {
                    stream = std::istringstream (rotLine);
                    std::getline(stream, val, ',');
                    pose.second.yaw = std::stod(val);// + M_PI/2.0;
                    std::getline(stream, val, ',');
                    pose.second.roll = std::stod(val);
                    std::getline(stream, val, ',');
                    pose.second.pitch = -1* std::stod(val);
                } else {
                    pose.second = firstRot;
                }
                m_poseVecs[nodeIdx].push_back(pose);
                if (isFirst) {
                    firstPos = pose.first;
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
                ElLoc loc = ElLoc(x - ((dims.width-1)/2),y - ((dims.height-1)/2));
//                ElLoc loc = ElLoc(x,y);
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
            weightssets.push_back(calcSteervec(eulers[idx], dims));
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

    Eigen::Quaterniond VecToQuat(const Vector3D &vec) {
//        Vector3D normVec = vec /  vec.GetLength();
//
//        double cos_theta = normVec.y;
//        double theta = std::acos(cos_theta);
//        double w = std::cos(theta / 2);
//        double sin_theta_over_2 = std:: sin(theta / 2);
//        double x = normVec.x * sin_theta_over_2;
//        double y = normVec.y * sin_theta_over_2;
//        double z = normVec.z * sin_theta_over_2;
//        return Eigen::Quaterniond(w,x,y,z);
        Eigen::Vector3d toVec(vec.x,vec.y,vec.z);
        Eigen::Vector3d fromVec(1,0,0);

        //        std::cout << "FROMTO" << fromVec << " " <<toVec << std::endl;
        return Eigen::Quaterniond::FromTwoVectors(fromVec, toVec);
    }

    Eigen::Quaterniond EulerToQuat(const Euler& euler) {
        double cr = std::cos(euler.roll * 0.5);
        double sr = std::sin(euler.roll * 0.5);
        double cp = std::cos(euler.pitch * 0.5);
        double sp = std::sin(euler.pitch * 0.5);
        double cy = std::cos(euler.yaw * 0.5);
        double sy = std::sin(euler.yaw * 0.5);

        //Pitch axis points left
        double w =  cy * cr * cp + sy * sr * sp;
        double x =  cy * sr * cp + sy * cr * sp;
        double y = -cy * cr * sp + sy * sr * cp;
        double z = -cy * sr * sp + sy * cr * cp;
        return Eigen::Quaterniond(w, x, y, z);
    }

    Euler QuatToEuler(const Eigen::Quaterniond &q) {
        int i = 1;
        int j = 0;
        int k = 2;

        int sign = (i - j) * (j - k) * (k - i) / 2;

        Euler eul;
        double eps = 1e-7;
        int thecase;

        double a = q.w() - q.x();
        double b = q.y() + q.z() * sign;
        double c = q.x() + q.w();
        double d = q.z() * sign - q.y();
//        std::cout << "abcd" << a << " " << b << " " << c << " " << d << std::endl;

        eul.roll = 2 * std::atan2(std::hypot(c, d), std::hypot(a, b));

        if (std::abs(eul.roll) <= eps)
            thecase = 1;
        else if (std::abs(eul.roll - M_PI) <= eps)
            thecase = 2;
        else
            thecase = 0;


        double half_sum = std::atan2(b, a);
        double half_diff = std::atan2(d, c);

        if (thecase == 0) {
            eul.pitch = half_sum - half_diff;
            eul.yaw = half_sum + half_diff;
        } else {
            eul.yaw = 0;
            if (thecase == 1)
                eul.pitch = 2 * half_sum;
            else
                eul.pitch = 2 * half_diff * -1;
        }

        eul.yaw *= sign;
        eul.roll -= M_PI / 2;

        if (eul.yaw < -M_PI)
            eul.yaw += 2 * M_PI;
        else if (eul.yaw > M_PI)
            eul.yaw -= 2 * M_PI;
        if (eul.roll < -M_PI)
            eul.roll += 2 * M_PI;
        else if (eul.roll > M_PI)
            eul.roll -= 2 * M_PI;
        if (eul.pitch < -M_PI)
            eul.pitch += 2 * M_PI;
        else if (eul.pitch > M_PI)
            eul.pitch -= 2 * M_PI;

        eul.pitch *= -1; //Pitch axis points left

        if (thecase != 0) {
            std::cout << "GIMBAL LOCK" << std::endl;
        }
        return eul;

        //
//        //OK
//        Euler eul;
//        // roll (x-axis rotation)
//        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
//        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
//        eul.roll = std::atan2(sinr_cosp, cosr_cosp);
//
//        // pitch (y-axis rotation)
//        double sinp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
//        double cosp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
//        eul.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
//
//        // yaw (z-axis rotation)
//        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
//        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
//        eul.yaw = std::atan2(siny_cosp, cosy_cosp);
//        return eul;
    }


    UV EulerToUv(const Euler& eul) {
        return UV(std::cos(eul.pitch)*std::sin(eul.yaw), std::sin(eul.pitch));
    }

    Euler uvToEuler(const UV& uv) {
        return Euler(std::atan2(uv.u, sqrt(1 - std::pow(uv.u, 2) - std::pow(uv.v, 2))),0, std::asin(uv.v));
    }

    VectorCplx fillFrom(const VectorCplx& source, const MatrixXb& selection) {
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

    void multiplyWhere(VectorCplx& v, const MatrixXb& selection, cplx multiplier) {
        for (int xSource = 0; xSource < selection.cols(); xSource++) {
            for (int ySource = 0; ySource < selection.rows(); ySource++) {
                if (selection(xSource,ySource) == true) {
                    v(xSource*selection.cols()+ySource) *= multiplier;
                }
            }
        }
    }

    VectorCplx normalize(const VectorCplx& vec) {
        return vec * 1 / sqrt(vec.size());
    }

    double calcSteervecEl(Euler euler, ElLoc elloc) {
        return M_PI * 2 * SPACING * (std::sin(euler.yaw) * elloc.x * std::cos(euler.pitch) + elloc.y * std::sin(euler.pitch));
    }
}
