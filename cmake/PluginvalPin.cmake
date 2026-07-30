# Pinned pluginval release, consumed by Task P0.6 (local install + validation gate) and the
# P0.7 CI workflow's download step. Only the Windows asset is pinned: the windows CI job is
# the only one that builds cnpg_plugin and runs pluginval against it (the ubuntu job is
# dsp/-only and never produces a VST3).
#
# Version: v1.0.4 -- confirmed the latest stable pluginval release as of pin date 2026-07-30
# via `git ls-remote --tags https://github.com/Tracktion/pluginval.git`
# (https://github.com/Tracktion/pluginval/releases/tag/v1.0.4).
set(CNPG_PLUGINVAL_VERSION "1.0.4")
set(CNPG_PLUGINVAL_WINDOWS_URL
    "https://github.com/Tracktion/pluginval/releases/download/v${CNPG_PLUGINVAL_VERSION}/pluginval_Windows.zip")

# SHA-256 of pluginval_Windows.zip for v1.0.4 -- verified by Task P0.6.
# Computed 2026-07-30 via PowerShell `Get-FileHash -Algorithm SHA256` over the asset downloaded
# directly from the official Tracktion GitHub release page (URL above); pluginval.exe extracted
# from that archive reports `pluginval - 1.0.4`, confirming the asset matches the pinned version.
set(CNPG_PLUGINVAL_WINDOWS_SHA256 "C08E61CE3B96DB41636F8EC7E76F4C7E2C13EBDAC7FA1B5A1F52B4F32EC715AB")
