

export class VisClient {
    constructor() {
        this.canvas = document.getElementById("canvas")
        this.ctx = this.canvas.getContext("2d")
        this.width = this.canvas.width
        this.height = this.canvas.height
        this.anchorPoint = {x: this.width / 2, y: this.height / 2}
        // draw cube with 3d acceleration
        this.cube = {
            x: this.width / 2,
            y: this.height / 2,
            z: 0,
            alpha: 0,
            beta: 0,
            gamma: 0,
        }
    }

    drawCube() {
        this.ctx.clearRect(0, 0, this.width, this.height)
        this.ctx.save()
        this.ctx.translate(this.anchorPoint.x, this.anchorPoint.y)
        this.ctx.rotate(this.cube.alpha)
        this.ctx.rotate(this.cube.beta)
        this.ctx.rotate(this.cube.gamma)
        this.ctx.beginPath()
        this.ctx.moveTo(-50, -50)
        this.ctx.lineTo(50, -50)
        this.ctx.lineTo(50, 50)
        this.ctx.lineTo(-50, 50)
        this.ctx.closePath()
        this.ctx.stroke()
        this.ctx.restore()
    }


    updateCube(x, y, z, alpha, beta, gamma) {
        this.cube.x = x
        this.cube.y = y
        this.cube.z = z
        this.cube.alpha = alpha
        this.cube.beta = beta
        this.cube.gamma = gamma
        this.drawCube()
    }
  


}