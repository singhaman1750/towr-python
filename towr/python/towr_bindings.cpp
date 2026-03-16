#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <ifopt/problem.h>
#include <towr/models/robot_model.h>
#include <towr/nlp_formulation.h>
#include <towr/parameters.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/state.h>

#ifdef TOWR_PYTHON_HAS_IPOPT
#include <ifopt/ipopt_solver.h>
#endif

namespace py = pybind11;
using namespace towr;

namespace {

py::dict SampleSolution(const SplineHolder& solution, double sample_dt)
{
    py::list times;
    py::list base_linear;
    py::list base_angular;
    py::list ee_motion;
    py::list ee_force;
    py::list ee_contact;

    const double total_time = solution.base_linear_ ? solution.base_linear_->GetTotalTime() : 0.0;
    const double dt = sample_dt > 0.0 ? sample_dt : 0.2;

    for (double t = 0.0; t <= total_time + 1e-5; t += dt) {
        times.append(t);
        base_linear.append(solution.base_linear_->GetPoint(t).p());
        base_angular.append(solution.base_angular_->GetPoint(t).p());

        py::list motion_at_t;
        py::list force_at_t;
        py::list contact_at_t;
        for (size_t ee = 0; ee < solution.ee_motion_.size(); ++ee) {
            motion_at_t.append(solution.ee_motion_.at(ee)->GetPoint(t).p());
            force_at_t.append(solution.ee_force_.at(ee)->GetPoint(t).p());
            contact_at_t.append(solution.phase_durations_.at(ee)->IsContactPhase(t));
        }

        ee_motion.append(motion_at_t);
        ee_force.append(force_at_t);
        ee_contact.append(contact_at_t);
    }

    py::dict sampled;
    sampled["times"] = times;
    sampled["base_linear"] = base_linear;
    sampled["base_angular"] = base_angular;
    sampled["ee_motion"] = ee_motion;
    sampled["ee_force"] = ee_force;
    sampled["ee_contact"] = ee_contact;
    sampled["total_time"] = total_time;
    sampled["sample_dt"] = dt;
    return sampled;
}

py::dict BuildProblemAndSolve(NlpFormulation& formulation,
                              double max_cpu_time,
                              const std::string& jacobian_approximation,
                              double sample_dt,
                              bool print_nlp)
{
#ifdef TOWR_PYTHON_HAS_IPOPT
    ifopt::Problem nlp;
    SplineHolder solution;

    for (auto& variable_set : formulation.GetVariableSets(solution)) {
        nlp.AddVariableSet(variable_set);
    }
    for (auto& constraint_set : formulation.GetConstraints(solution)) {
        nlp.AddConstraintSet(constraint_set);
    }
    for (auto& cost_set : formulation.GetCosts()) {
        nlp.AddCostSet(cost_set);
    }

    ifopt::IpoptSolver solver(true);
    solver.SetOption("jacobian_approximation", jacobian_approximation);
    solver.SetOption("max_cpu_time", max_cpu_time);
    try {
        solver.Solve(nlp);
    } catch (const std::exception& error) {
        throw std::runtime_error(std::string("IPOPT threw an exception while solving: ") + error.what());
    }

    const int return_status = solver.GetReturnStatus();
    if (return_status != 0 && return_status != 1) {
        std::ostringstream message;
        message << "IPOPT failed to find a solution. Return code: " << return_status;
        throw std::runtime_error(message.str());
    }

    if (print_nlp) {
        nlp.PrintCurrent();
    }

    py::dict result;
    result["iterations"] = nlp.GetIterationCount();
    result["return_status"] = return_status;
    result["variable_values"] = nlp.GetVariableValues();
    result["solver_total_wallclock_time"] = solver.GetTotalWallclockTime();
    result["sampled"] = SampleSolution(solution, sample_dt);
    return result;
#else
    (void)formulation;
    (void)max_cpu_time;
    (void)jacobian_approximation;
    (void)sample_dt;
    (void)print_nlp;
    throw std::runtime_error(
        "towr_cpp was built without IPOPT solver support. Rebuild ifopt with BUILD_IPOPT=ON, "
        "install IPOPT, then rebuild towr_cpp.");
#endif
}

} // namespace

PYBIND11_MODULE(towr_cpp, m) {
    m.doc() = "Minimal TOWR Python bindings";

    py::enum_<Dx>(m, "Dx")
        .value("Pos", Dx::kPos)
        .value("Vel", Dx::kVel)
        .value("Acc", Dx::kAcc)
        .value("Jerk", Dx::kJerk)
        .export_values();

    py::enum_<RobotModel::Robot>(m, "RobotType")
        .value("Monoped", RobotModel::Robot::Monoped)
        .value("Biped", RobotModel::Robot::Biped)
        .value("Hyq", RobotModel::Robot::Hyq)
        .value("Anymal", RobotModel::Robot::Anymal)
        .export_values();

    py::enum_<HeightMap::Direction>(m, "TerrainDirection")
        .value("Normal", HeightMap::Direction::Normal)
        .value("Tangent1", HeightMap::Direction::Tangent1)
        .value("Tangent2", HeightMap::Direction::Tangent2)
        .export_values();

    py::class_<Node>(m, "Node")
        .def(py::init<int>(), py::arg("dim") = 0)
        .def_property(
            "p",
            [](const Node& node) { return node.p(); },
            [](Node& node, const Eigen::VectorXd& value) { node.at(kPos) = value; })
        .def_property(
            "v",
            [](const Node& node) { return node.v(); },
            [](Node& node, const Eigen::VectorXd& value) { node.at(kVel) = value; });

    py::class_<BaseState>(m, "BaseState")
        .def(py::init<>())
        .def_readwrite("lin", &BaseState::lin)
        .def_readwrite("ang", &BaseState::ang);

    py::class_<RobotModel>(m, "RobotModel")
        .def(py::init<RobotModel::Robot>())
        .def("mass", [](const RobotModel& model) {
            return model.dynamic_model_ ? model.dynamic_model_->m() : 0.0;
        })
        .def("num_ee", [](const RobotModel& model) {
            return model.kinematic_model_ ? model.kinematic_model_->GetNumberOfEndeffectors() : 0;
        })
        .def("max_dev_from_nominal", [](const RobotModel& model) {
            return model.kinematic_model_
                ? model.kinematic_model_->GetMaximumDeviationFromNominal()
                : Eigen::Vector3d::Zero();
        })
        .def("nominal_stance", [](const RobotModel& model) {
            return model.kinematic_model_
                ? model.kinematic_model_->GetNominalStanceInBase()
                : KinematicModel::EEPos{};
        });

    py::class_<Parameters>(m, "Parameters")
        .def(py::init<>())
        .def_readwrite("ee_phase_durations", &Parameters::ee_phase_durations_)
        .def_readwrite("ee_in_contact_at_start", &Parameters::ee_in_contact_at_start_)
        .def_readwrite("bound_phase_duration", &Parameters::bound_phase_duration_)
        .def("optimize_phase_durations", &Parameters::OptimizePhaseDurations)
        .def("is_optimize_timings", &Parameters::IsOptimizeTimings)
        .def("total_time", &Parameters::GetTotalTime);

    py::class_<HeightMap, std::shared_ptr<HeightMap>>(m, "HeightMap")
        .def("height", &HeightMap::GetHeight)
        .def("normalized_basis", &HeightMap::GetNormalizedBasis)
        .def("friction_coeff", &HeightMap::GetFrictionCoeff);

    py::class_<FlatGround, HeightMap, std::shared_ptr<FlatGround>>(m, "FlatGround")
        .def(py::init<double>(), py::arg("height") = 0.0)
        .def("height", &FlatGround::GetHeight);

    py::class_<NlpFormulation>(m, "NlpFormulation")
        .def(py::init<>())
        .def_readwrite("initial_base", &NlpFormulation::initial_base_)
        .def_readwrite("final_base", &NlpFormulation::final_base_)
        .def_readwrite("initial_ee_W", &NlpFormulation::initial_ee_W_)
        .def_readwrite("model", &NlpFormulation::model_)
        .def_readwrite("terrain", &NlpFormulation::terrain_)
        .def_readwrite("params", &NlpFormulation::params_);

        m.def("has_ipopt_solver", []() {
    #ifdef TOWR_PYTHON_HAS_IPOPT
        return true;
    #else
        return false;
    #endif
        });

        m.def("solve_nlp",
          &BuildProblemAndSolve,
          py::arg("formulation"),
          py::arg("max_cpu_time") = 20.0,
          py::arg("jacobian_approximation") = "exact",
          py::arg("sample_dt") = 0.2,
          py::arg("print_nlp") = false);
}
