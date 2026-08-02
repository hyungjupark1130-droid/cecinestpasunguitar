# cnpg_calibrate -- bridge tuning verification across the (note x rate x admittance) grid (Task P2.7;
# docs/plan.md section P2.7 as amended by docs/decisions/0007-bridge-tuning-compensation.md).
#
# Links cnpg_dsp ONLY -- no JUCE, no Catch2 -- so it builds under the linux-dsp-only preset exactly
# like cnpg_dsp/cnpg_tests/cnpg_bench/cnpg_render do (the naming registry, docs/plan.md section 1.5).
#
# It shares tests/support/SpectralAnalysis.cpp with cnpg_tests rather than carrying its own estimator,
# and that is the point rather than a convenience: docs/plan.md section 4.5 mandates ONE estimator
# recipe, and a verification harness measuring the instrument with a second implementation of it
# could disagree with the gate for reasons that were about the estimator. That file has no Catch2
# dependency, so sharing it costs this target nothing.

add_executable(cnpg_calibrate
    CalibrateMain.cpp
    ${CMAKE_SOURCE_DIR}/tests/support/SpectralAnalysis.cpp
)

target_link_libraries(cnpg_calibrate PRIVATE cnpg_dsp)

# Lets CalibrateMain.cpp include the shared estimator as "support/SpectralAnalysis.h" -- the same
# spelling cnpg_tests uses.
target_include_directories(cnpg_calibrate PRIVATE ${CMAKE_SOURCE_DIR}/tests)

cnpg_set_warnings(cnpg_calibrate)
