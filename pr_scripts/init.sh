git fetch
git checkout develop
git merge --ff-only
# set hooks for commit etc -> update commit hooks,

regex="^[a-z0-9!#\$%&'*+/=?^_\`{|}~-]+(\.[a-z0-9!#$%&'*+/=?^_\`{|}~-]+)*@([a-z0-9]([a-z0-9-]*[a-z0-9])?\.)+[a-z0-9]([a-z0-9-]*[a-z0-9])?\$"

if [[ $(git config --global user.email) =~ $regex ]] || [[ $(git config --local user.email) =~ $regex ]] ; then
    echo "OK"
else
    echo "not OK"
fi

if [[ $(git config --global user.global) != "" ]] || [[ $(git config --local user.name) != "" ]] ; then
    echo "name OK"
else
    echo "name not OK"
fi

echo "some advices BUILD_WITH_TIDY"

echo branch name?
read varname

git checkout -b $varname

# git push -u origin


# run tests
