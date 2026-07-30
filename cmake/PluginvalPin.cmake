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

# SHA-256 of pluginval_Windows.zip for v1.0.4 -- NOT YET VERIFIED.
# GitHub's release API reports no digest for this asset, and Tracktion's release notes don't
# publish one either (both checked 2026-07-30). Downloading and hashing the binary is
# explicit-permission-required and out of scope for this task; Task P0.6 ("Install pluginval
# ... on the dev machine; record version") is the task that actually fetches the binary, so it
# owns computing this hash and replacing the placeholder below. Do not trust this value, and
# do not wire an integrity check against it, until P0.6 closes that out.
set(CNPG_PLUGINVAL_WINDOWS_SHA256 "UNVERIFIED-PENDING-P0.6")
