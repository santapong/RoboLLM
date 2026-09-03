# RoboLLM · UR5e VLA sim bed references

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../../README.md) · [Bed README](README.md) · [Specification](SDD.md) · [Notices](NOTICES.md) · [Documentation](../../docs/README.md)

Every external tool, model, dataset and paper the bed builds on, as BibTeX.
Keys follow the arXiv auto-export style used in the `xembodiment-lit`
workspace; entries copied from an upstream `CITATION.cff` or README keep the
upstream key. License and attribution obligations are in
[`NOTICES.md`](NOTICES.md).

## Simulators and models

```bibtex
@inproceedings{todorov2012mujoco,
  title     = {MuJoCo: A physics engine for model-based control},
  author    = {Todorov, Emanuel and Erez, Tom and Tassa, Yuval},
  booktitle = {2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},
  pages     = {5026--5033},
  year      = {2012},
  organization = {IEEE},
  doi       = {10.1109/IROS.2012.6386109}
}

@software{menagerie2022github,
  author = {Zakka, Kevin and Tassa, Yuval and {MuJoCo Menagerie Contributors}},
  title = {{MuJoCo Menagerie: A collection of high-quality simulation models for MuJoCo}},
  url = {http://github.com/google-deepmind/mujoco_menagerie},
  year = {2022},
}

@software{omnisim2026,
  title   = {OmniSim: an agent-facing robotics simulator},
  author  = {OmniLink},
  version = {8.1.6},
  year    = {2026},
  month   = {8},
  url     = {https://www.omnilink-agents.com/omnisim},
  license = {Apache-2.0},
  note    = {From the repository CITATION.cff. The build evaluated on 1 Sep 2026 was v8.1.17 at commit 19ea166, github.com/omnilink-tech/omnisim}
}

@article{michel2004cyberbotics,
  title   = {Cyberbotics Ltd. Webots\textsuperscript{TM}: Professional Mobile Robot Simulation},
  author  = {Michel, Olivier},
  journal = {International Journal of Advanced Robotic Systems},
  volume  = {1},
  number  = {1},
  pages   = {39--42},
  year    = {2004},
  note    = {OmniSim is a derivative of Webots R2025a (Cyberbotics, Apache-2.0)}
}
```

## Tooling

```bibtex
@software{Zakka_Mink_Python_inverse_2026,
  author = {Zakka, Kevin},
  title = {{Mink: Python inverse kinematics based on MuJoCo}},
  year = {2026},
  month = feb,
  version = {1.1.0},
  url = {https://github.com/kevinzakka/mink},
  license = {Apache-2.0}
}

@misc{yi2025viser,
  title         = {Viser: Imperative, Web-based 3D Visualization in Python},
  author        = {Brent Yi and Chung Min Kim and Justin Kerr and Gina Wu and Rebecca Feng and Anthony Zhang and Jonas Kulhanek and Hongsuk Choi and Yi Ma and Matthew Tancik and Angjoo Kanazawa},
  year          = {2025},
  eprint        = {2507.22885},
  archivePrefix = {arXiv},
  url           = {https://arxiv.org/abs/2507.22885}
}

@software{mjviser2026github,
  author  = {{The mjlab Developers}},
  title   = {mjviser: Web-based MuJoCo viewer powered by Viser},
  url     = {https://github.com/mujocolab/mjviser},
  version = {0.0.14},
  year    = {2026},
  license = {Apache-2.0}
}
```

## Robot learning stack

```bibtex
@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Meftah, Khalil and Ellerbach, Maxime and Moss, Jess and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}

@inproceedings{cadenelerobot,
  title={LeRobot: An Open-Source Library for End-to-End Robot Learning},
  author={Cadene, Remi and Alibert, Simon and Capuano, Francesco and Aractingi, Michel and Zouitine, Adil and Kooijmans, Pepijn and Choghari, Jade and Russi, Martino and Pascal, Caroline and Palma, Steven and Shukor, Mustafa and Moss, Jess and Soare, Alexander and Aubakirova, Dana and Lhoest, Quentin and Gallou\'edec, Quentin and Wolf, Thomas},
  booktitle={The Fourteenth International Conference on Learning Representations},
  year={2026},
  url={https://arxiv.org/abs/2602.22818}
}

@misc{shukor2025smolvla,
  title         = {SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics},
  author        = {Mustafa Shukor and Dana Aubakirova and Francesco Capuano and Pepijn Kooijmans and Steven Palma and Adil Zouitine and Michel Aractingi and Caroline Pascal and Martino Russi and Andres Marafioti and Simon Alibert and Matthieu Cord and Thomas Wolf and Remi Cadene},
  year          = {2025},
  eprint        = {2506.01844},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2506.01844},
  note          = {Weights: huggingface.co/lerobot/smolvla_base (license tag absent on the card as of 3 Sep 2026); base VLM: HuggingFaceTB/SmolVLM2-500M-Video-Instruct, Apache-2.0}
}

@misc{smolvlm2_500m_video_instruct,
  title        = {SmolVLM2-500M-Video-Instruct},
  author       = {{Hugging Face TB}},
  howpublished = {\url{https://huggingface.co/HuggingFaceTB/SmolVLM2-500M-Video-Instruct}},
  year         = {2025},
  note         = {Apache-2.0}
}
```

## Datasets and benchmarks

```bibtex
@misc{BerkeleyUR5Website,
    title = {Berkeley {UR5} Demonstration Dataset},
    author = {Lawrence Yunliang Chen and Simeon Adebola and Ken Goldberg},
    howpublished = {https://sites.google.com/view/berkeley-ur5/home},
}

@misc{lerobot_berkeley_autolab_ur5,
  title        = {lerobot/berkeley\_autolab\_ur5: LeRobot v3.0 conversion of the Berkeley UR5 Demonstration Dataset},
  author       = {{The LeRobot Team}},
  howpublished = {\url{https://huggingface.co/datasets/lerobot/berkeley_autolab_ur5}},
  note         = {CC-BY-4.0. 1,000 episodes, 97,939 frames, 5 fps, three 480x640 cameras, state[8], action[7]. Accessed 3 Sep 2026}
}

@misc{embodimentcollaboration2025openxembodimentroboticlearning,
      title={Open X-Embodiment: Robotic Learning Datasets and RT-X Models},
      author={{Embodiment Collaboration} and others},
      year={2025},
      eprint={2310.08864},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2310.08864},
      note={Full author list in the xembodiment-lit workspace, paper/references.bib, same key}
}

@misc{liu2023libero,
  title         = {LIBERO: Benchmarking Knowledge Transfer for Lifelong Robot Learning},
  author        = {Bo Liu and Yifeng Zhu and Chongkai Gao and Yihao Feng and Qiang Liu and Yuke Zhu and Peter Stone},
  year          = {2023},
  eprint        = {2306.03310},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2306.03310},
  note          = {Optional calibration run only (SDD phase P2b); packaged for LeRobot as hf-libero, MIT code, CC-BY-4.0 data}
}
```

## Design rules (read 3 Sep 2026, SDD §14)

```bibtex
@misc{laskey2017dart,
  title         = {DART: Noise Injection for Robust Imitation Learning},
  author        = {Michael Laskey and Jonathan Lee and Roy Fox and Anca Dragan and Ken Goldberg},
  year          = {2017},
  eprint        = {1703.09327},
  archivePrefix = {arXiv},
  primaryClass  = {cs.LG},
  url           = {https://arxiv.org/abs/1703.09327},
  note          = {CoRL 2017. Used for: execute noisy, record clean (Alg. 1); noise level matters (Fig. 5); HSR grasping 49 to 79 percent}
}

@misc{zhang2025actionchunking,
  title         = {Action Chunking and Exploratory Data Collection Yield Exponential Improvements in Behavior Cloning for Continuous Control},
  author        = {Thomas T. Zhang and Daniel Pfrommer and Chaoyi Pan and Nikolai Matni and Max Simchowitz},
  year          = {2025},
  eprint        = {2507.09061},
  archivePrefix = {arXiv},
  primaryClass  = {cs.LG},
  url           = {https://arxiv.org/abs/2507.09061},
  note          = {Used for: Practice 2 (clean labels, clean+noisy mixture, larger noise ok), Thm 1 (executed chunk length), robomimic tool\_hang sweep}
}

@misc{feng2026actionspace,
  title         = {Demystifying Action Space Design for Robotic Manipulation Policies},
  author        = {Yuchun Feng and Jinliang Zheng and Zhihao Wang and Dongxiu Liu and Jianxiong Li and Jiangmiao Pang and Tai Wang and Xianyuan Zhan},
  year          = {2026},
  eprint        = {2602.23408},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2602.23408},
  note          = {13,000+ real rollouts. Used for: delta over absolute, chunk-wise over step-wise delta, shorter horizon for delta, task space for transfer, 6x6 grid protocol}
}

@misc{yan2026zeta,
  title         = {ZETA: A Controlled Study of Zero-Shot Cross-Embodiment VLA Transfer for Tabletop Manipulation},
  author        = {Mi Yan and Wenhao Zhang and Zhiqi Zhang and Yu Peng and Tangxinyu Wang and others},
  year          = {2026},
  eprint        = {2609.02546},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2609.02546},
  note          = {Used for: gripper-frame (EEF-delta) actions and states (Table 3, Table 4), strict vs pretrain-exposed zero-shot, progress score, arm-only shift category}
}

@misc{zhao2026proprio,
  title         = {How Should Vision-Language-Action Models Use Proprioceptive State?},
  author        = {Yiren Zhao and Ziyang Chen and Ziyang Rao and Pengteng Li and He Zhang and Weiyu Guo and Yandong Guo and Rushi Dai},
  year          = {2026},
  eprint        = {2608.03052},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2608.03052},
  note          = {Used for: modest state gains, joint vs EE state secondary, short history to the action head, state-shortcut risk}
}

@misc{fan2026safevlabench,
  title         = {SafeVLA-Bench: A Benchmark for the Success-Safety Gap in Vision-Language-Action Models},
  author        = {Jialiang Fan and Weizhe Xu and Oleg Sokolsky and Insup Lee and Fanxin Kong},
  year          = {2026},
  eprint        = {2606.00773},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2606.00773},
  note          = {Used for: (SR, Safety, SBU, VSI) quadruple, clipped-linear severity depth, Wilson intervals, spec-family design}
}

@misc{lei2026cotraining,
  title         = {A Mechanistic Analysis of Sim-and-Real Co-Training in Generative Robot Policies},
  author        = {Yu Lei and Minghuan Liu and Abhiram Maddukuri and Zhenyu Jiang and Yuke Zhu},
  year          = {2026},
  eprint        = {2604.13645},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2604.13645},
  note          = {Used for: balanced mixing ratios (0.016 to 0.3), representation alignment vs domain discernibility, domain label}
}

@misc{qiu2025mpcsafegil,
  title         = {Safety-Aware Imitation Learning via MPC-Guided Disturbance Injection},
  author        = {Le Qiu and Yusuf Umut Ciftci and Somil Bansal},
  year          = {2025},
  eprint        = {2508.03129},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://arxiv.org/abs/2508.03129},
  note          = {Counter-evidence: random Gaussian or uniform noise gave no gain on quadruped and F1Tenth navigation; adversarial disturbance did}
}
```
