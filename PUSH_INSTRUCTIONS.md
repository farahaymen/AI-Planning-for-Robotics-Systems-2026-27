# Pushing this repository to GitHub

The repository is already initialised with one commit. You need to create the
remote and push.

## 1. Create the repository on GitHub

Create an **empty** repository. Do not let GitHub add a README, .gitignore or
licence, or the first push will conflict.

- Owner: your course organisation if you have one, otherwise your account
- Name: `arc-course`
- Visibility: **private** for now. Make it public once you are happy with it;
  students fork from it, so it must be readable to them by then.

## 2. Push

From the repository root:

```bash
git remote add origin https://github.com/YOUR-ORG/arc-course.git
git branch -M main
git push -u origin main
```

If you use SSH rather than HTTPS:

```bash
git remote add origin git@github.com:YOUR-ORG/arc-course.git
```

## 3. Point the provisioning at it

Stage 50 clones this repository into the VM. Either edit the default in
`scripts/provision/50-workspace.sh`, or pass it at run time:

```bash
sudo ARC_REPO=https://github.com/YOUR-ORG/arc-course.git ./provision.sh 50
```

## 4. Turn on the CI

`.github/workflows/ci.yml` runs the 100 offline tests on every push. Nothing to
configure; it runs on the first push. A green tick means the mapping model, the
particle filter, the arena generator and the competition scoring all still
behave. It does not test anything requiring ROS, which no CI runner has.

## 5. Large files

`.gitignore` already excludes bags, `.mcap`, `.vdi`, `.ova` and saved maps.
Keep it that way. A three-minute rosbag is tens of megabytes and will exceed
GitHub's limits quickly.

If you want to version the reference recording for Lab 4, use Git LFS:

```bash
git lfs install
git lfs track "*.mcap"
git add .gitattributes
```

Otherwise distribute it through the VLE and leave the repository light.

## 6. Branch protection, once students have access

Protect `main` so students fork and open pull requests rather than pushing to
your course repository. Settings → Branches → Add rule → require a pull request
before merging.
