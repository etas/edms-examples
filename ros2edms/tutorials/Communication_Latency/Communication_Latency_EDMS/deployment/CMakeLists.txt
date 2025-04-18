
yaaa_add_visualization(
  NAME Interprocess.activity_graph
)

yaaa_add_visualization(
  NAME Intraprocess.activity_graph
)

yaaa_add_deploy(
  NAME DeploymentLinux.deploy
  MEASUREMENT_CONFIG
      ENABLE_INSTANCE_CONVERSION
)
