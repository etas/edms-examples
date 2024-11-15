
yaaa_add_visualization(
  NAME HelloWorld.activity_graph
)

yaaa_add_deploy(
  NAME Deployment.deploy
  MEASUREMENT_CONFIG
      ENABLE_INSTANCE_CONVERSION
)
