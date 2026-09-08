#include "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/test_mpcc_architecture_snapshot.cpp"
namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
{
TEST(AuthorityFailureBundleBefore, SecondFailureMustKeepItsOwnPublishedArtifact)
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  auto source = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::Cruise);
  // The generic snapshot fixture carries an Overtake execution side. Cruise
  // has neutral intent geometry even when a dynamic-obstacle homotopy exists.
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  source.request.current_steering_rad = 0.0;
  source.request.current_response_steering_rad = 0.0;
  source.request.initial_state[3] = 2.0;
  execution::ExecutionArtifact artifact;
  artifact.identity = source.identity;
  artifact.prediction_origin_sec = source.control_prediction_origin_sec;
  artifact.publication_interval_sec = 0.025;
  artifact.completed_sec = source.identity.snapshot_sec + 0.01;
  artifact.course_progress_origin_m = source.course_progress_origin_m;
  artifact.course_frame = {
    std::make_shared<const std::vector<mpc_stage_geometry::CourseFrameKnot>>(
      source.wall_course_frame_knots), source.course_progress_origin_m};
  artifact.wheelbase_m = 1.0;
  artifact.maximum_abs_steering_rad = 0.5;
  artifact.maximum_abs_steering_rate_radps = 1.0;
  artifact.physical_global_tolerance = 1e-6;
  artifact.maximum_constraint_violation = 1e-8;
  artifact.maximum_normalized_constraint_violation = 0.1;
  artifact.predicted_states = {
    {0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 2.0, 0.2, 0.0, 0.0}};
  artifact.semantic_initial_state = artifact.predicted_states.front();
  artifact.control_stages = {{0.0, 0.0, 2.0, 0.1, 0.0, 4.0, -3.0, 1.37, 0.0}};
  artifact.nominal_path_distance_m = {0.0, 0.2};
  artifact.lateral_lower_m = {-1.0, -1.0};
  artifact.lateral_upper_m = {1.0, 1.0};
  ASSERT_EQ(execution::validate(artifact), execution::RejectReason::None);
  PublicationEvidence publication;
  publication.failure_decision_id = 55U;
  publication.failure_interaction_fingerprint = 12345U;
  publication.failure_observation_sec = 12.61;
  publication.failure_control_origin_sec = 12.74;
  publication.source_kind = "exact-executed";
  publication.publication_decision_id = 54U;
  publication.publication_control_origin_sec = 12.65;
  publication.publication_artifact_elapsed_sec = 0.03712345678901234;

  const auto root = output_root("distinct-authority-loss-before");
  std::filesystem::remove_all(root);
  auto world = source;
  world.identity.sequence = 55U;
  const auto first_world = record_proof_failure(world, PipelineStage::PhysicalProof,
    "startup-normal-authority-unavailable", "first boundary", root);
  ASSERT_EQ(first_world.status, RecordStatus::Written);
  const auto first_source = record_published_execution(source, artifact, publication,
    "first boundary", root);
  ASSERT_EQ(first_source.status, RecordStatus::Written);
  // Different current failure and a different actual artifact, same normal intent.
  world.identity.sequence = 4428U;
  const auto second_world = record_proof_failure(world, PipelineStage::PhysicalProof,
    "terminal-contingency-unavailable", "second boundary", root);
  ASSERT_EQ(second_world.status, RecordStatus::Written);
  source.identity.sequence = 3834U;
  artifact.identity = source.identity;
  publication.failure_decision_id = 4428U;
  const auto second_source = record_published_execution(source, artifact, publication,
    "second boundary", root);
  EXPECT_EQ(second_source.status, RecordStatus::Written)
    << "A written new failure must not lose its actual source to an unrelated first bucket: "
    << second_source.detail;
}
}
