# Bibliografia formal de apoyo

Este archivo recoge las referencias base para el estado del arte del articulo. La lista esta preparada en estilo numerado tipo IEEE y puede adaptarse a la plantilla final del congreso o revista.

## Referencias recomendadas

[1] I. Lenz, H. Lee, and A. Saxena, "Deep learning for detecting robotic grasps," *The International Journal of Robotics Research*, vol. 34, no. 4-5, pp. 705-724, 2015. doi: 10.1177/0278364914549607.

[2] D. Morrison, P. Corke, and J. Leitner, "Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach," in *Robotics: Science and Systems XIV*, 2018. doi: 10.15607/RSS.2018.XIV.021.

[3] A. Depierre, E. Dellandrea, and L. Chen, "Jacquard: A Large Scale Dataset for Robotic Grasp Detection," in *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2018, pp. 3511-3516. doi: 10.1109/IROS.2018.8593950.

[4] J. Mahler, J. Liang, S. Niyaz, M. Laskey, R. Doan, X. Liu, J. Aparicio Ojea, and K. Goldberg, "Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics," in *Robotics: Science and Systems XIII*, 2017. doi: 10.15607/RSS.2017.XIII.058.

[5] H.-S. Fang, C. Wang, M. Gou, and C. Lu, "GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping," in *Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, 2020, pp. 11444-11453. doi: 10.1109/CVPR42600.2020.01146.

[6] K. He, X. Zhang, S. Ren, and J. Sun, "Deep Residual Learning for Image Recognition," in *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, 2016, pp. 770-778. doi: 10.1109/CVPR.2016.90.

[7] J. Pineau, P. Vincent-Lamarre, K. Sinha, V. Lariviere, A. Beygelzimer, F. d'Alche-Buc, E. Fox, and H. Larochelle, "Improving Reproducibility in Machine Learning Research (A Report from the NeurIPS 2019 Reproducibility Program)," *Journal of Machine Learning Research*, vol. 22, no. 164, pp. 1-20, 2021.

[8] O. E. Gundersen and S. Kjensmo, "State of the Art: Reproducibility in Artificial Intelligence," in *Proceedings of the AAAI Conference on Artificial Intelligence*, vol. 32, no. 1, pp. 1644-1651, 2018. doi: 10.1609/aaai.v32i1.11503.

## Uso dentro del articulo

- [1] Fundamenta Cornell y la representacion rectangular de agarres.
- [2] Situa GG-CNN como referencia de prediccion densa y control reactivo en grasping.
- [3] Situa Jacquard como dataset sintetico RGB-D de gran escala y con orientacion reproducible.
- [4] Situa Dex-Net 2.0 como referencia de aprendizaje de calidad de agarre con datos sinteticos y metricas analiticas.
- [5] Situa GraspNet-1Billion como benchmark RGB-D de gran escala para grasping general.
- [6] Justifica el uso de ResNet-18 como arquitectura residual de referencia.
- [7] y [8] respaldan el framing de reproducibilidad experimental.

## BibTeX base

```bibtex
@article{lenz2015deep,
  author = {Lenz, Ian and Lee, Honglak and Saxena, Ashutosh},
  title = {Deep learning for detecting robotic grasps},
  journal = {The International Journal of Robotics Research},
  volume = {34},
  number = {4-5},
  pages = {705--724},
  year = {2015},
  doi = {10.1177/0278364914549607}
}

@inproceedings{morrison2018closing,
  author = {Morrison, Douglas and Corke, Peter and Leitner, Jurgen},
  title = {Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach},
  booktitle = {Robotics: Science and Systems XIV},
  year = {2018},
  doi = {10.15607/RSS.2018.XIV.021}
}

@inproceedings{depierre2018jacquard,
  author = {Depierre, Amaury and Dellandrea, Emmanuel and Chen, Liming},
  title = {Jacquard: A Large Scale Dataset for Robotic Grasp Detection},
  booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages = {3511--3516},
  year = {2018},
  doi = {10.1109/IROS.2018.8593950}
}

@inproceedings{mahler2017dexnet,
  author = {Mahler, Jeffrey and Liang, Jacky and Niyaz, Sherdil and Laskey, Michael and Doan, Richard and Liu, Xinyu and Aparicio Ojea, Juan and Goldberg, Ken},
  title = {Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics},
  booktitle = {Robotics: Science and Systems XIII},
  year = {2017},
  doi = {10.15607/RSS.2017.XIII.058}
}

@inproceedings{fang2020graspnet,
  author = {Fang, Hao-Shu and Wang, Chenxi and Gou, Minghao and Lu, Cewu},
  title = {GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping},
  booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages = {11444--11453},
  year = {2020},
  doi = {10.1109/CVPR42600.2020.01146}
}

@inproceedings{he2016resnet,
  author = {He, Kaiming and Zhang, Xiangyu and Ren, Shaoqing and Sun, Jian},
  title = {Deep Residual Learning for Image Recognition},
  booktitle = {Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
  pages = {770--778},
  year = {2016},
  doi = {10.1109/CVPR.2016.90}
}

@article{pineau2021reproducibility,
  author = {Pineau, Joelle and Vincent-Lamarre, Philippe and Sinha, Koustuv and Lariviere, Vincent and Beygelzimer, Alina and d'Alche-Buc, Florence and Fox, Emily and Larochelle, Hugo},
  title = {Improving Reproducibility in Machine Learning Research (A Report from the NeurIPS 2019 Reproducibility Program)},
  journal = {Journal of Machine Learning Research},
  volume = {22},
  number = {164},
  pages = {1--20},
  year = {2021}
}

@inproceedings{gundersen2018reproducibility,
  author = {Gundersen, Odd Erik and Kjensmo, Sigbjorn},
  title = {State of the Art: Reproducibility in Artificial Intelligence},
  booktitle = {Proceedings of the AAAI Conference on Artificial Intelligence},
  volume = {32},
  number = {1},
  pages = {1644--1651},
  year = {2018},
  doi = {10.1609/aaai.v32i1.11503}
}
```
