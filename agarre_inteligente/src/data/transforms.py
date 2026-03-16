"""Transformaciones para RGB/RGB-D en entrenamiento y validacion."""

from __future__ import annotations

import torchvision.transforms as T


def get_train_transforms(image_size: int = 224, augmentation: bool = False):
    ops = [T.Resize((image_size, image_size))]
    if augmentation:
        ops.extend(
            [
                T.RandomHorizontalFlip(p=0.5),
                T.RandomRotation(15),
                T.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2),
            ]
        )
    ops.extend([T.ToTensor()])
    return T.Compose(ops)


def get_val_transforms(image_size: int = 224):
    return T.Compose([T.Resize((image_size, image_size)), T.ToTensor()])
