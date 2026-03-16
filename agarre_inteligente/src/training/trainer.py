"""Entrenador reproducible para experimentos EXP1..EXP4."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import pandas as pd
import torch

from src.evaluation.evaluator import Evaluator
from src.utils.checkpoint import CheckpointManager


@dataclass
class EpochLog:
    epoch: int
    train_loss: float
    val_loss: float
    val_success: float
    val_iou: float
    val_angle_deg: float


class Trainer:
    def __init__(
        self,
        model: torch.nn.Module,
        train_loader,
        val_loader,
        optimizer,
        criterion,
        device: str,
        run_dir: str | Path,
        val_csv_path: str | Path | None = None,
    ):
        self.model = model
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.optimizer = optimizer
        self.criterion = criterion
        self.device = device
        self.run_dir = Path(run_dir)
        self.val_csv_path = val_csv_path
        self.ckpt = CheckpointManager(self.run_dir / "checkpoints")

    def train(self, epochs: int) -> tuple[pd.DataFrame, int]:
        self.model.to(self.device)
        logs = []
        best_epoch = 1
        best_success = -1.0

        for epoch in range(1, epochs + 1):
            self.model.train()
            batch_losses = []
            for x, y, _ in self.train_loader:
                x = x.to(self.device)
                y = y.to(self.device)
                self.optimizer.zero_grad(set_to_none=True)
                pred = self.model(x)
                loss = self.criterion(pred, y)
                if not torch.isfinite(loss):
                    continue
                loss.backward()
                self.optimizer.step()
                batch_losses.append(float(loss.item()))

            if not batch_losses:
                batch_losses = [float("nan")]
            train_loss = float(sum(batch_losses) / max(len(batch_losses), 1))

            evaluator = Evaluator(self.model, self.val_loader, self.device, csv_path=self.val_csv_path)
            val_res = evaluator.evaluate(self.criterion)

            row = EpochLog(
                epoch=epoch,
                train_loss=train_loss,
                val_loss=val_res.val_loss,
                val_success=val_res.val_success,
                val_iou=val_res.val_iou,
                val_angle_deg=val_res.val_angle_deg,
            )
            logs.append(asdict(row))

            state = {
                "epoch": epoch,
                "model_state_dict": self.model.state_dict(),
                "optimizer_state_dict": self.optimizer.state_dict(),
                "metrics": asdict(row),
            }
            self.ckpt.save_last(state)

            if row.val_success > best_success:
                best_success = row.val_success
                best_epoch = epoch
                self.ckpt.save_best(state)

        df = pd.DataFrame(logs)
        df.to_csv(self.run_dir / "metrics.csv", index=False)
        return df, best_epoch
