import { cloneDeep } from 'lodash';
import { Vector2 } from 'three';

/**
 * 采用旋转卡壳法，由凸包点集生成最小面积矩形，时间复杂度为O(n)，算法详情见：https://www.geometrictools.com/GTE/Mathematics/MinimumAreaBox2.h
 * @param points 输入为逆时针排序的凸多边形的顶点，输入点不能包含三个连续共线的点。
 * @returns 最小面积矩形的宽、高、中心点及旋转角度
 */

type Box = {
  u: Vector2[];
  index: number[];
  sqrLenU0: number;
  area: number;
};
type AngleInfo = {
  sinThetaSqr: number;
  indexOfBox: number;
};

export const rotatingCaliper = (hull: [number, number][]) => {
  const vertices = hull.map((point) => new Vector2(point[0], point[1]));
  const visited = new Array(vertices.length).fill(false);

  let minBox = smallestBox(vertices.length - 1, 0, vertices);
  visited[minBox.index[0]] = true;

  const box = cloneDeep(minBox);
  const angleInfo: AngleInfo[] = new Array(4);
  const angleNum = { value: 0 };
  for (let i = 0; i < vertices.length; ++i) {
    if (!computeAngles(vertices, box, angleInfo, angleNum)) {
      break;
    }
    const sort = sortAngles(angleInfo, angleNum);
    if (!updateSupport(angleInfo, angleNum, sort, vertices, visited, box)) {
      break;
    }
    if (box.area <= minBox.area) {
      minBox = cloneDeep(box);
    }
  }
  const normalU0 = minBox.u[0].clone().normalize();
  const normalU1 = minBox.u[1].clone().normalize();
  const lastSupport = vertices[minBox.index[3]].clone();
  const origin = vertices[minBox.index[0] - 1 === -1 ? vertices.length - 1 : minBox.index[0] - 1].clone();
  const maxV = normalU0.clone().multiplyScalar(lastSupport.clone().sub(origin).dot(normalU0));
  const rectVertice = maxV.clone().add(origin);

  const diff = [
    vertices[minBox.index[1]].clone().sub(vertices[minBox.index[3]]),
    vertices[minBox.index[2]].clone().sub(vertices[minBox.index[0]]),
  ];
  const width = Math.abs(normalU0.dot(diff[0]));
  const height = Math.abs(normalU1.dot(diff[1]));
  const angle = Math.atan2(minBox.u[0].y, minBox.u[0].x);

  const vw = normalU0.clone().multiplyScalar(width * 0.5);
  const vh = normalU1.clone().multiplyScalar(height * 0.5);
  const center = rectVertice.clone().add(vw).add(vh).toArray();

  return {
    width,
    height,
    center,
    angle,
  };
};

const perpE = new Vector2(0, 0);
function computeAngles(vertices: Vector2[], box: Box, angleInfo: AngleInfo[], angleNum: { value: number }): boolean {
  angleNum.value = 0;
  for (let k0 = 3, k1 = 0; k1 < 4; k0 = k1++) {
    if (box.index[k0] !== box.index[k1]) {
      const d = k0 & 2 ? box.u[k0 & 1].negate() : box.u[k0 & 1];
      const j0 = box.index[k0];
      let j1 = j0 + 1;
      if (j1 === vertices.length) {
        j1 = 0;
      }
      const e = vertices[j1].clone().sub(vertices[j0]);
      perpE.set(-e.y, e.x);
      const dp = perpE.dot(d);
      const dsqrslen = d.dot(d);
      const esqrlen = e.dot(e);
      const sinThetaSqr = (dp * dp) / (esqrlen * dsqrslen);
      angleInfo[angleNum.value] = { sinThetaSqr, indexOfBox: k0 };
      angleNum.value++;
    }
  }
  return angleNum.value > 0;
}

function sortAngles(angleInfo: AngleInfo[], angleNum: { value: number }): number[] {
  if (angleNum.value === 0) {
    return [0, 1, 2, 3];
  }
  const sort = angleInfo.map((a, index) => index);
  sort.sort((a, b) => {
    // 处理支持点重合的情况：若最小角度落在了重合的支持点上，要更新更逆时针的支持点，而不是第一个
    if (angleNum.value < 4 && angleInfo[a].sinThetaSqr === angleInfo[b].sinThetaSqr) {
      // 把 indexOfBox 更大的排在前面
      return angleInfo[b].indexOfBox - angleInfo[a].indexOfBox;
    } else {
      return angleInfo[a].sinThetaSqr - angleInfo[b].sinThetaSqr;
    }
  });
  return sort;
}

// 更新支持点
const u1 = new Vector2();
function updateSupport(
  angleInfo: AngleInfo[],
  angleNum: { value: number },
  sort: number[],
  vertices: Vector2[],
  visited: boolean[],
  box: Box
): boolean {
  const minAngle = angleInfo[sort[0]];

  let parallel = 0;
  // 替换那些达到最小值的边的支撑顶点与边的其他端点的角度。
  for (let k = 0; k < angleNum.value; ++k) {
    const curAngle = angleInfo[sort[k]];
    if (curAngle.sinThetaSqr === minAngle.sinThetaSqr) {
      box.index[curAngle.indexOfBox] = box.index[curAngle.indexOfBox] + 1;
      if (box.index[curAngle.indexOfBox] === vertices.length) {
        box.index[curAngle.indexOfBox] = 0;
      }
      parallel++;
    }
  }
  const bottom = box.index[minAngle.indexOfBox];
  if (visited[bottom]) {
    // 已经处理过这个多边形边。
    return false;
  }

  for (let k = 0; k < parallel; k++) {
    visited[box.index[angleInfo[sort[k]].indexOfBox]] = true;
  }

  // Cycle the vertices so that the bottom support occurs sinThetaSqr.
  const nextIndex = Array(4).fill(0);
  for (let k = 0; k < 4; ++k) {
    nextIndex[k] = box.index[(minAngle.indexOfBox + k) % 4];
  }
  box.index = nextIndex;

  // Compute the box axis directions.
  const j1 = box.index[0];
  let j0 = j1 - 1;
  if (j0 < 0) {
    j0 = vertices.length - 1;
  }

  box.u[0] = vertices[j1].clone().sub(vertices[j0]);
  box.u[1] = u1.set(-box.u[0].y, box.u[0].x);
  box.sqrLenU0 = box.u[0].lengthSq();

  // Compute the box area.
  const diff = [
    vertices[box.index[1]].clone().sub(vertices[box.index[3]]),
    vertices[box.index[2]].clone().sub(vertices[box.index[0]]),
  ];
  box.area = (box.u[0].dot(diff[0]) * box.u[1].dot(diff[1])) / box.sqrLenU0;
  return true;
}

function smallestBox(i0: number, i1: number, vertices: Vector2[]): Box {
  const box = {
    u: [],
    index: [i1, i1, i1, i1],
    sqrLenU0: 0,
    area: 0,
  } as Box;
  box.u[0] = new Vector2().subVectors(vertices[i1], vertices[i0]);
  box.u[1] = new Vector2(-box.u[0].y, box.u[0].x);
  box.sqrLenU0 = box.u[0].dot(box.u[0]);

  const origin = vertices[i1];
  const support = [new Vector2(0, 0), new Vector2(-0, -0), new Vector2(-0, -0), new Vector2(0, 0)];

  const diff = new Vector2();
  const v = new Vector2();
  vertices.forEach((vertex, i) => {
    diff.subVectors(vertex, origin);
    v.set(box.u[0].dot(diff), box.u[1].dot(diff));

    if (v.x > support[1].x || (v.x === support[1].x && v.y > support[1].y)) {
      box.index[1] = i;
      support[1].set(v.x, v.y);
    }

    if (v.y > support[2].y || (v.y === support[2].y && v.x < support[2].x)) {
      box.index[2] = i;
      support[2].set(v.x, v.y);
    }

    if (v.x < support[3].x || (v.x === support[3].x && v.y < support[3].y)) {
      box.index[3] = i;
      support[3].set(v.x, v.y);
    }
  });

  const scaledWidth = support[1].x - support[3].x;
  const scaledHeight = support[2].y;
  box.area = (scaledWidth * scaledHeight) / box.sqrLenU0;
  return box;
}
