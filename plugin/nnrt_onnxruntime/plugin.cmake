stepit_declare_plugin(NAME nnrt_onnxruntime DEPENDS
    nnrt_base
)
if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|aarch64|arm64")
  stepit_plugin_mark_unbuildable(
      "Requires x86_64 or arm64/aarch64 architectures, got '${CMAKE_SYSTEM_PROCESSOR}'."
  )
  return()
endif ()
