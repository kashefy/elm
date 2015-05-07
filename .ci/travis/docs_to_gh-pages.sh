#!/bin/bash
# Update Doxygen documentation on github-pages to reflect latest changes in master
# credits:
#  https://github.com/miloyip/rapidjson/blob/master/travis-doxygen.sh
#  http://stackoverflow.com/questions/23277391/pushing-to-github-from-travis-ci
#  https://github.com/robotology/icub-iaitabletop/blob/master/.travis/push-doxygen-to-gh-pages.sh
#  http://rickfoosusa.blogspot.de/2011/10/howto-use-doxygen-with-github.html
# Author: kashefy
set -e # exit with nonzero exit code if anything fails

# skip builds triggered by pull-requests
[ "${TRAVIS_PULL_REQUEST}" = "false" ] || \
skip "Skipping docs generation builds triggered by pull-requests."

# limit to specific branches
#[ "${TRAVIS_BRANCH}" = "master" ] || \
#skip "Limit docs generation to pushes to 'master' branch (current branch is: ${TRAVIS_BRANCH})."

cd $TRAVIS_BUILD_DIR
echo -e "Generate doxygen docs"
doxygen ./docs/doxygenConf.txt > /dev/null

git config --global user.email "travis@travis-ci.org"
git config --global user.name "travis-ci"
git clone --quiet --branch=gh-pages https://${GH_TOKEN}@github.com/kashefy/elm gh-pages > /dev/null

cd gh-pages
if [ -d "docs" ]; then
  git rm -rf docs
fi
mkdir -p docs/
cp -Rf $TRAVIS_BUILD_DIR/docs/html ./docs/html
git add -f .
git commit -m "[gh-pages] [docs] auto-push updated doxygen docs on successful travis build $TRAVIS_BUILD_NUMBER to gh-pages"
git push -fq origin gh-pages
echo -e "Published doxygen docs to gh-pages."
