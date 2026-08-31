stepit_declare_plugin(NAME nnrt_torchjit DEPENDS
    nnrt_base
)
find_package(Torch QUIET)
if (NOT Torch_FOUND)
  stepit_plugin_mark_unbuildable("Missing libtorch.")
  return()
endif ()
