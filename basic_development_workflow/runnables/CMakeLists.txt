yaaa_add_runnable(
  NAME Producer.runnable
  SOURCES src/producer.cpp
  CPP_INCLUDE_DIRS runnables_include
)

yaaa_add_runnable(
  NAME Consumer.runnable
  SOURCES src/consumer.cpp
  CPP_INCLUDE_DIRS runnables_include
)
