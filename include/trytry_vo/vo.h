//
// Created by heihei on 2021/12/1.
//

#ifndef TRYTRY_VO_VO_H
#define TRYTRY_VO_VO_H

#include "trytry_vo/config.h"
#include "trytry_vo/frontend.h"

#include <vector>
#include <Eigen/Dense>


/**
 * 这是对外的接口
 */
class Vo {
public:
    Vo();

    ~Vo();

    bool Init();

    bool Run();

    bool Step();

    Status getFrontStatus() const;

private:

    Frontend frontend;

    std::shared_ptr<Images> images_ptr;

    Camera camera;

};


#endif //TRYTRY_VO_VO_H
