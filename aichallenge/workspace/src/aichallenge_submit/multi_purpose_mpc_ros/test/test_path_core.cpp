#include "multi_purpose_mpc_ros/path_core.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace path_core = multi_purpose_mpc_ros::path_core;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kHeader =
  "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n";

class TempCsv
{
public:
  explicit TempCsv(const std::string & content)
  {
    static std::atomic<unsigned long> sequence{0};
    const auto timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
      ("multi_purpose_mpc_path_core_" + std::to_string(timestamp) + "_" +
      std::to_string(sequence.fetch_add(1)) + ".csv");

    std::ofstream output(path_);
    if (!output.is_open()) {
      throw std::runtime_error("failed to create temporary CSV: " + path_.string());
    }
    output << content;
    if (!output.good()) {
      throw std::runtime_error("failed to write temporary CSV: " + path_.string());
    }
  }

  ~TempCsv()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  TempCsv(const TempCsv &) = delete;
  TempCsv & operator=(const TempCsv &) = delete;

  const std::string path() const {return path_.string();}

private:
  std::filesystem::path path_;
};

void expect_csv_error(
  const std::string & content, const std::vector<std::string> & expected_fragments)
{
  const TempCsv csv(content);
  try {
    static_cast<void>(path_core::load_reference_path_csv(csv.path()));
    FAIL() << "Expected strict CSV loader to reject input";
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find(csv.path()), std::string::npos) << message;
    for (const auto & fragment : expected_fragments) {
      EXPECT_NE(message.find(fragment), std::string::npos) << message;
    }
  }
}

path_core::ReferencePathPoint point(
  const double s, const double x, const double y, const double psi, const double kappa,
  const double velocity, const double acceleration)
{
  return {s, x, y, psi, kappa, velocity, acceleration};
}

}  // namespace

TEST(PathCoreAngle, WrapsZeroPiAndFivePiWithDirectionPreserved)
{
  EXPECT_DOUBLE_EQ(path_core::wrap_to_pi(0.0), 0.0);
  EXPECT_NEAR(path_core::wrap_to_pi(kPi), kPi, 1e-15);
  EXPECT_NEAR(path_core::wrap_to_pi(-kPi), -kPi, 1e-15);
  EXPECT_NEAR(path_core::wrap_to_pi(5.0 * kPi), kPi, 1e-15);
  EXPECT_NEAR(path_core::wrap_to_pi(-5.0 * kPi), -kPi, 1e-15);
}

TEST(PathCoreAngle, WrapsTrajectoryYawDifferencesAcrossBothSidesOfSeam)
{
  const double forward_difference = 3.112 - (-3.003);
  const double reverse_difference = -3.003 - 3.112;
  EXPECT_NEAR(path_core::wrap_to_pi(forward_difference), -0.168185307179586, 1e-12);
  EXPECT_NEAR(path_core::wrap_to_pi(reverse_difference), 0.168185307179586, 1e-12);
}

TEST(PathCoreCsv, LoadsStrictSevenColumnsAndPreservesEveryField)
{
  const TempCsv csv(
    std::string("  \xEF\xBB\xBF  s_m , x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2 \n") +
    "0, 1.25,-2.5,3.0,-0.125,4.5,-1.5\n"
    "0.25,2.25,-3.5,-3.0,0.25,5.5,1.5\n");

  const auto points = path_core::load_reference_path_csv(csv.path());

  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].s_m, 0.0);
  EXPECT_DOUBLE_EQ(points[0].x_m, 1.25);
  EXPECT_DOUBLE_EQ(points[0].y_m, -2.5);
  EXPECT_DOUBLE_EQ(points[0].psi_rad, 3.0);
  EXPECT_DOUBLE_EQ(points[0].kappa_radpm, -0.125);
  EXPECT_DOUBLE_EQ(points[0].vx_mps, 4.5);
  EXPECT_DOUBLE_EQ(points[0].ax_mps2, -1.5);
  EXPECT_DOUBLE_EQ(points[1].s_m, 0.25);
  EXPECT_DOUBLE_EQ(points[1].ax_mps2, 1.5);
}

TEST(PathCoreCsv, RejectsMissingAndDuplicateHeaders)
{
  expect_csv_error(
    "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps\n0,1,2,3,4,5\n1,2,3,4,5,6\n",
    {"row 1", "column 'ax_mps2'", "required header is missing"});
  expect_csv_error(
    "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,vx_mps\n0,1,2,3,4,5,6\n1,2,3,4,5,6,7\n",
    {"row 1", "column 'vx_mps'", "duplicate header"});
}

TEST(PathCoreCsv, RejectsShortRowsWithExpectedColumnDiagnostic)
{
  expect_csv_error(
    std::string(kHeader) + "0,1,2,3,4,5\n1,2,3,4,5,6,7\n",
    {"row 2", "column 'ax_mps2'", "value '<missing>'", "expected 7"});
}

TEST(PathCoreCsv, RejectsBlankRowsAndExtraColumns)
{
  expect_csv_error(
    std::string(kHeader) +
    "0,0,0,0,0,1,0\n\n1,1,0,0,0,1,0\n",
    {"row 3", "column '<record>'", "value '<blank>'", "blank data rows"});
  expect_csv_error(
    "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,extra\n"
    "0,0,0,0,0,1,0,unused\n1,1,0,0,0,1,0,unused\n",
    {"row 1", "column 'extra'", "exactly 7 columns"});
  expect_csv_error(
    std::string(kHeader) +
    "0,0,0,0,0,1,0,unexpected\n1,1,0,0,0,1,0\n",
    {"row 2", "column '<column-8>'", "value 'unexpected'", "expected 7"});
}

TEST(PathCoreCsv, RejectsBadPartialAndEmptyNumbers)
{
  expect_csv_error(
    std::string(kHeader) + "0,bad,2,3,4,5,6\n1,2,3,4,5,6,7\n",
    {"row 2", "column 'x_m'", "value 'bad'", "invalid numeric value"});
  expect_csv_error(
    std::string(kHeader) + "0,1.0suffix,2,3,4,5,6\n1,2,3,4,5,6,7\n",
    {"row 2", "column 'x_m'", "value '1.0suffix'", "partially converted"});
  expect_csv_error(
    std::string(kHeader) + "0,,2,3,4,5,6\n1,2,3,4,5,6,7\n",
    {"row 2", "column 'x_m'", "numeric value is empty"});
}

TEST(PathCoreCsv, RejectsNanAndInfinity)
{
  expect_csv_error(
    std::string(kHeader) + "0,nan,2,3,4,5,6\n1,2,3,4,5,6,7\n",
    {"row 2", "column 'x_m'", "value 'nan'", "must be finite"});
  expect_csv_error(
    std::string(kHeader) + "0,1,2,3,4,inf,6\n1,2,3,4,5,6,7\n",
    {"row 2", "column 'vx_mps'", "value 'inf'", "must be finite"});
}

TEST(PathCoreCsv, RejectsNonIncreasingArcLength)
{
  expect_csv_error(
    std::string(kHeader) + "0,1,2,3,4,5,6\n0,2,3,4,5,6,7\n",
    {"row 3", "column 's_m'", "strictly increasing"});
}

TEST(PathCoreCsv, RejectsDegenerateAndNonFiniteDerivedSegmentLengths)
{
  expect_csv_error(
    std::string(kHeader) +
    "0,0,0,0,0,1,0\n1,0.0000005,0,0,0,1,0\n",
    {"row 3", "column '<segment-length>'", "must be more than"});
  expect_csv_error(
    std::string(kHeader) +
    "0,-1e308,0,0,0,1,0\n1,1e308,0,0,0,1,0\n",
    {"row 3", "column '<segment-length>'", "derived segment length must be finite"});
}

TEST(PathCoreCsv, UsesExclusiveMinimumSegmentLengthBoundary)
{
  expect_csv_error(
    std::string(kHeader) +
    "0,0,0,0,0,1,0\n1,0.000001,0,0,0,1,0\n",
    {"row 3", "column '<segment-length>'", "must be more than"});

  const TempCsv csv(
    std::string(kHeader) +
    "0,0,0,0,0,1,0\n1,0.0000010000000001,0,0,0,1,0\n");
  EXPECT_EQ(path_core::load_reference_path_csv(csv.path()).size(), 2U);
}

TEST(PathCoreCsv, RejectsInsufficientPointCount)
{
  expect_csv_error(
    std::string(kHeader) + "0,1,2,3,4,5,6\n",
    {"row 3", "column '<record>'", "at least 2 data rows"});
}

TEST(PathCoreCircular, RemovesClosureDuplicateAsWholeRecord)
{
  std::vector<path_core::ReferencePathPoint> points{
    point(0.0, 0.0, 0.0, 0.1, 0.2, 3.0, 4.0),
    point(1.0, 1.0, 0.0, 1.1, 1.2, 13.0, 14.0),
    point(2.0, 0.0, 1.0, 2.1, 2.2, 23.0, 24.0),
    point(3.0, 0.0005, 0.0, 9.1, 9.2, 93.0, 94.0)};

  EXPECT_TRUE(path_core::normalize_circular_endpoint(points, 0.001));

  ASSERT_EQ(points.size(), 3U);
  EXPECT_DOUBLE_EQ(points.front().s_m, 0.0);
  EXPECT_DOUBLE_EQ(points.front().psi_rad, 0.1);
  EXPECT_DOUBLE_EQ(points.back().s_m, 2.0);
  EXPECT_DOUBLE_EQ(points.back().x_m, 0.0);
  EXPECT_DOUBLE_EQ(points.back().y_m, 1.0);
  EXPECT_DOUBLE_EQ(points.back().psi_rad, 2.1);
  EXPECT_DOUBLE_EQ(points.back().kappa_radpm, 2.2);
  EXPECT_DOUBLE_EQ(points.back().vx_mps, 23.0);
  EXPECT_DOUBLE_EQ(points.back().ax_mps2, 24.0);
}

TEST(PathCoreCircular, UsesInclusiveClosureToleranceBoundary)
{
  const auto make_points = [](const double endpoint_x) {
      return std::vector<path_core::ReferencePathPoint>{
        point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        point(1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        point(2.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0),
        point(3.0, endpoint_x, 0.0, 0.0, 0.0, 1.0, 0.0)};
    };

  auto at_tolerance = make_points(0.001);
  EXPECT_TRUE(path_core::normalize_circular_endpoint(at_tolerance, 0.001));
  EXPECT_EQ(at_tolerance.size(), 3U);

  auto above_tolerance = make_points(std::nextafter(0.001, 1.0));
  EXPECT_FALSE(path_core::normalize_circular_endpoint(above_tolerance, 0.001));
  EXPECT_EQ(above_tolerance.size(), 4U);
}

#ifdef MULTI_PURPOSE_MPC_ROS_SOURCE_DIR
TEST(PathCoreCircular, RemovesOneEndpointFromConfiguredFinalVer3Trajectory)
{
  const std::filesystem::path csv_path =
    std::filesystem::path(MULTI_PURPOSE_MPC_ROS_SOURCE_DIR) /
    "env/final_ver3/traj_mincurv.csv";
  auto points = path_core::load_reference_path_csv(csv_path.string());
  const std::size_t raw_size = points.size();

  EXPECT_TRUE(path_core::normalize_circular_endpoint(points, 0.001));
  EXPECT_EQ(points.size() + 1U, raw_size);
}
#endif

TEST(PathCoreCircular, KeepsDistinctEndpoint)
{
  std::vector<path_core::ReferencePathPoint> points{
    point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(2.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0)};

  EXPECT_FALSE(path_core::normalize_circular_endpoint(points, 0.001));
  EXPECT_EQ(points.size(), 3U);
}

TEST(PathCoreCircular, RejectsTooFewUniquePointsWithoutMutatingInput)
{
  std::vector<path_core::ReferencePathPoint> points{
    point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)};

  EXPECT_THROW(path_core::normalize_circular_endpoint(points, 0.001), std::invalid_argument);
  EXPECT_EQ(points.size(), 3U);
}

TEST(PathCoreCircular, RejectsNonClosureConsecutiveZeroLengthEdge)
{
  std::vector<path_core::ReferencePathPoint> points{
    point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(2.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(3.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0)};

  EXPECT_THROW(path_core::normalize_circular_endpoint(points, 0.001), std::invalid_argument);
}

TEST(PathCoreCircular, RejectsNonClosureNearZeroLengthEdge)
{
  std::vector<path_core::ReferencePathPoint> points{
    point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(2.0, 1.0 + path_core::kMinimumSegmentLengthM * 0.5, 0.0, 0.0, 0.0, 1.0, 0.0),
    point(3.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0)};

  EXPECT_THROW(path_core::normalize_circular_endpoint(points, 0.001), std::invalid_argument);
}

TEST(PathCoreSubdivision, UsesCeilAndAtLeastOneSubdivision)
{
  EXPECT_EQ(path_core::subdivision_count(0.999, 0.25), 4U);
  EXPECT_EQ(path_core::subdivision_count(1.0, 0.25), 4U);
  EXPECT_EQ(path_core::subdivision_count(0.001, 0.25), 1U);
  EXPECT_EQ(path_core::subdivision_count(0.0, 0.25), 1U);
}

TEST(PathCoreSubdivision, RejectsInvalidInputs)
{
  EXPECT_THROW(path_core::subdivision_count(-0.1, 0.25), std::invalid_argument);
  EXPECT_THROW(path_core::subdivision_count(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(path_core::subdivision_count(1.0, -0.25), std::invalid_argument);
  EXPECT_THROW(
    path_core::subdivision_count(std::numeric_limits<double>::infinity(), 0.25),
    std::invalid_argument);
  EXPECT_THROW(
    path_core::subdivision_count(1.0, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    path_core::subdivision_count(std::numeric_limits<double>::quiet_NaN(), 0.25),
    std::invalid_argument);
  EXPECT_THROW(
    path_core::subdivision_count(1.0, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(PathCoreSubdivision, RejectsAResultOutsideSizeTWithoutFloatingPointConversionOverflow)
{
  const double first_unrepresentable_count =
    std::ldexp(1.0, std::numeric_limits<std::size_t>::digits);
  ASSERT_TRUE(std::isfinite(first_unrepresentable_count));
  EXPECT_THROW(
    path_core::subdivision_count(first_unrepresentable_count, 1.0), std::overflow_error);
}
