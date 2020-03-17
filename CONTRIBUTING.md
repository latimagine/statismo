Contribution Types
==================

Found a Bug?
------------

If you find a bug in the source code, you can help us by submitting an issue to our [GitHub Repository](https://github.com/kenavolic/statismo/issues). Even better, you can [submit a Pull Request](#Submit-a-Pull-Request) with a fix.

> :information_source: Please check if the issue it not already in the backlog first.

Want a New Feature?
-------------------

If you are missing a feature, first submit an issue to our [GitHub Repository](https://github.com/kenavolic/statismo/issues) to chat about your proposal and be sure it will be used!

Then, everythng is clarified, you can [submit a Pull Request](#Submit-a-Pull-Request) with your new feature.

Submission Guidelines
=====================

Submit an Issue
---------------

You can file new issues by selecting from our new issue templates and filling out the issue template.

Submit a Pull Request
---------------------

First fork the repository to your github account by clicking the ```Fork``` button on the project github main page.

Then clone the **the forked repo**:
```
> git clone git@github.com:username/statismo.git
```

> :warning: Check our [git rules](#Git-Rules) before processing the next steps!

From the root of **the forked repo**, run the workspace initialization script:
```
bash ./workspace/pr_init.sh
```

This script would make your local repo ready. It will ask for a branch name
that should comply with the branch naming convention.

> :warning: Check our [coding rules](#Coding-Rules) before processing the next steps!

Then, enter your development cycle:
* Update the code following the [coding rules](#Coding-Rules),
* Check clang-format,
```
make format-check
# if there are formating issues, run
make format
```
* Run c++ tests locally,
```
ctest -C [Release|Debug]
```
* Run Python tests locally,
```
runVTKPythonTests[Release|Debug].sh
```
* Commit code according to [git rules](#Git-Rules).

At the end, push your code to **the forked repo**:
```
git push -u origin HEAD
```

Then, create a pull request on the forked repo to merge your work to the main
repo.

> :warning: The default target branch proposed by Github is ```master```. Update it to ```develop```!

The PR will be run against the CI system and checked by a reviewer.

Then you continue to work on the PR until it is accepted. Before the final merge,
a little bit of cleanup could be useful in your commit messages with local rebasing/rewording:
~~~
git commit --amend
git rebase HEAD~2
~~~

Git Rules
=========

### Commit Message

The format should be:
```
<type>: <subject>
<BLANK LINE>
<body> (optional)
<BLANK LINE>
<footer> (optional)
```

***type***

The type should be taken in this list:
* build : change in cmake configuration, compiler toolchain
* ci    : change to the ci scripts or utilities added for ci
* doc   : documentation
* feat  : new feature (class, routines)
* fix   : bug fix
* misc  : other stuff (use with care)
* perf  : performance enhancement
* sanity: code cleaning (warnings), refactoring
* format: code formatting
* test  : test enhancement

***subject***

The subject is in lowercase (even the first letter) and contains a succinct description of the change. Use the imperative, present tense
and do not termionate with a dot!

***body***

Use the imperative, present tense. The body should include the motivation for the change.

***footer***

The footer should contain a [closing reference to an issue](https://help.github.com/articles/closing-issues-via-commit-messages/) if any.

### Branching Model

The branching model is pretty standard (see [gitflow workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)):
 * The ```master``` branch is the stable one with merge only for new release
 * The ```develop``` branch is the main branch for development and PR
 * The features branch are forked from ```develop``` and must follow the [branch naming convention](#Branch-Naming)

### Branch Naming

The format is ```<type>/<id>```.

***type***

The type should be taken in this list:
* feat        : new feature PR
* fix         : issue fixing PR
* sanity      : cleaning or refactoring PR
* prerelease  : prerelase (do not use)
* release     : release (do not use)
* hotfix      : hotfix (do not use)

Coding Rules
============

### Prerequisites

You must install *clang-tidy* (>=6.0.0) and *clang-format* (>=6.0.0) before to ensure your configuration
mirrors the one in CI.

On Linux, make a symbolic link toward your installed version:
```
sudo ln -s /usr/bin/clang-tidy-6.0 /usr/bin/clang-tidy
```

### Project Configuration

Please check the [installation guide](#doc/md/INSTALL.md) for the building process.

The project must be set with the following options in dev mode:
 * BUILD_WITH_TIDY=ON
 * BUILD_WITH_WRAPPING=ON

### Coding Rules

As user guides are not read, we tried to automate the coding rules compliance:
* *clang-format* ensures code layout
* *clang-tidy* ensures code style
* *clang-tidy* and compiler options ensure code sanity

> :warning: The code style is different for each module. The ```core``` module
> complies with Statismo code style while ```ITK/VTK``` modules styles are close to
> ITK/VTK styles. Check the .clang-tidy at the root of each module for more details
> on the correct style or wait for compilation warning/error...
