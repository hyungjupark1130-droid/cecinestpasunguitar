#!/usr/bin/env bash
# .github/scripts/check-golden-commit.sh -- golden-regeneration commit guard (Task P0.7).
#
# tests/data/golden/ (landing in P1.4) holds committed float64 golden IRs used by the
# tight-tolerance regression tests (docs/plan.md section 1.6). Regenerating them is expected to
# happen deliberately, with a stated reason -- this script fails CI whenever a commit touches
# anything under tests/data/golden/ without carrying a `Regenerate-Goldens: <reason>` trailer in
# its commit message body. The guard is wired from P0 even though no golden files exist yet, so
# it is exercised (as a no-op: zero commits touch the not-yet-existing path) from the first CI
# run.
#
# Usage: check-golden-commit.sh [<base-sha>] [<head-sha>]
#   <base-sha>  Exclusive lower bound of the commit range to check. May be omitted, empty, or
#               the all-zero SHA (new-branch / force-push edge case, e.g. `git push --force` or
#               a branch's first push where GitHub reports `before` as 40 zeros) -- in any of
#               those cases only the single <head-sha> commit is checked.
#   <head-sha>  Inclusive upper bound of the commit range to check. Defaults to HEAD.
#
# Requires the checkout to have enough history to resolve <base-sha>..<head-sha> (i.e.
# actions/checkout with fetch-depth: 0, or at least deep enough to cover the pushed range).

set -euo pipefail

readonly GOLDEN_PATH_PREFIX="tests/data/golden/"
readonly TRAILER_KEY="Regenerate-Goldens"
readonly ZERO_SHA="0000000000000000000000000000000000000000"

base_sha="${1:-}"
head_sha="${2:-HEAD}"

if [[ -z "$base_sha" || "$base_sha" == "$ZERO_SHA" ]]; then
    range_description="$head_sha (single commit; no usable base SHA)"
    commits="$(git rev-list --max-count=1 "$head_sha")"
else
    range_description="${base_sha}..${head_sha}"
    commits="$(git rev-list "${base_sha}..${head_sha}")"
fi

if [[ -z "$commits" ]]; then
    echo "check-golden-commit: no commits in range ${range_description}; nothing to check."
    exit 0
fi

failures=0
checked=0

while IFS= read -r sha; do
    [[ -z "$sha" ]] && continue
    checked=$((checked + 1))

    changed_golden="$(git diff-tree --no-commit-id --name-only -r "$sha" -- "$GOLDEN_PATH_PREFIX")"
    if [[ -z "$changed_golden" ]]; then
        continue
    fi

    subject="$(git log -1 --format=%s "$sha")"
    message="$(git log -1 --format=%B "$sha")"

    if grep -qE "^${TRAILER_KEY}:[[:space:]]*[^[:space:]]" <<<"$message"; then
        echo "OK    $sha  $subject"
        continue
    fi

    echo "::error::Commit $sha ($subject) touches ${GOLDEN_PATH_PREFIX} without a '${TRAILER_KEY}: <reason>' trailer in its commit message."
    echo "  Changed golden paths:"
    sed 's/^/    /' <<<"$changed_golden"
    failures=$((failures + 1))
done <<<"$commits"

echo ""
echo "check-golden-commit: checked ${checked} commit(s) in range ${range_description}."

if [[ "$failures" -gt 0 ]]; then
    echo "check-golden-commit: ${failures} commit(s) touch ${GOLDEN_PATH_PREFIX} without the required trailer." >&2
    echo "" >&2
    echo "Add a trailer line to the offending commit's message body, e.g.:" >&2
    echo "" >&2
    echo "    Regenerate-Goldens: <why the reference output legitimately changed>" >&2
    echo "" >&2
    exit 1
fi

echo "check-golden-commit: all commits touching ${GOLDEN_PATH_PREFIX} (if any) carry a valid ${TRAILER_KEY} trailer."
exit 0
