#!/bin/bash
# Update Doxygen documentation on github-pages to reflect latest changes in master
# credits:
#  https://github.com/miloyip/rapidjson/blob/master/travis-doxygen.sh
#  http://stackoverflow.com/questions/23277391/pushing-to-github-from-travis-ci
#  https://github.com/robotology/icub-iaitabletop/blob/master/.travis/push-doxygen-to-gh-pages.sh
#  http://rickfoosusa.blogspot.de/2011/10/howto-use-doxygen-with-github.html
# Author: kashefy
set -e

# skip builds triggered by pull-requests
[ "${TRAVIS_PULL_REQUEST}" = "false" ] || \
skip "Skipping docs generation builds triggered by pull-requests."

# limit to specific branches
#[ "${TRAVIS_BRANCH}" = "master" ] || \
#skip "Limit docs generation to pushes to 'master' branch (current branch is: ${TRAVIS_BRANCH})."
