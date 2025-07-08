/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "types.h"

#include "../Core/array.h"
#include "../PathAlgos/RRT_PathFinder.h"
#include "../PathAlgos/SRRT_PathFinder.h"

void init_PathAlgos(pybind11::module& m) {
    pybind11::class_<rai::RRT_PathFinder, std::shared_ptr<rai::RRT_PathFinder>>(m, "RRT_PathFinder", "todo doc")
        .def(pybind11::init<>())
        .def("setProblem", &rai::RRT_PathFinder::setProblem, "", pybind11::arg("Configuration"))
        .def("setStartGoal", &rai::RRT_PathFinder::setStartGoal, "", pybind11::arg("starts"), pybind11::arg("goals"))
        .def("setExplicitCollisionPairs", &rai::RRT_PathFinder::setExplicitCollisionPairs, "only after setProblem", pybind11::arg("collisionPairs"))
        .def("solve", &rai::RRT_PathFinder::solve, "")
        .def("get_resampledPath", &rai::RRT_PathFinder::get_resampledPath, "");

    pybind11::class_<rai::SRRT_PathFinder, std::shared_ptr<rai::SRRT_PathFinder>>(m, "SRRT_PathFinder", "todo doc")
        .def(pybind11::init<>())
        // .def(pybind11::init<const rai::StringA&, const rai::StringA&>(), "",
        //     pybind11::arg("collision_pairs"),
        //     pybind11::arg("relevant_frame_names")
        // )
        .def("setInfo", &rai::SRRT_PathFinder::setInfo, "", pybind11::arg("collision_pairs"), pybind11::arg("relevant_frames"))
        .def("setStartGoal", &rai::SRRT_PathFinder::setStartGoal, "", pybind11::arg("starts"), pybind11::arg("goals"))
        .def("solve", &rai::SRRT_PathFinder::solve, "");
}

#endif
