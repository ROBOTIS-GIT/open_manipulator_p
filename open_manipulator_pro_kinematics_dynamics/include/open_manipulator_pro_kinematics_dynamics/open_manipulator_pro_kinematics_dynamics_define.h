/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_DEFINE_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_DEFINE_H_

namespace open_manipulator_pro
{

#define MAX_JOINT_ID  6
#define ALL_JOINT_ID  7

#define MAX_ITER      10

#define END_LINK      7

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_DEFINE_H_ */
