# 验证 DeepSeek API Key 是否可用
# 用法: .\verify-deepseek-api.ps1 -ApiKey "sk-你的密钥"
# 或:   $env:DEEPSEEK_API_KEY="sk-xxx"; .\verify-deepseek-api.ps1

param(
    [string]$ApiKey = $env:DEEPSEEK_API_KEY,
    [ValidateSet("deepseek-v4-flash", "deepseek-v4-pro")]
    [string]$Model = "deepseek-v4-flash"
)

if (-not $ApiKey) {
    Write-Host "错误: 请提供 API Key" -ForegroundColor Red
    Write-Host '  .\verify-deepseek-api.ps1 -ApiKey "sk-你的密钥"'
    exit 1
}

$body = @{
    model    = $Model
    messages = @(
        @{ role = "user"; content = "回复 OK 即可" }
    )
    stream   = $false
} | ConvertTo-Json -Depth 4

$headers = @{
    "Content-Type"  = "application/json"
    "Authorization" = "Bearer $ApiKey"
}

Write-Host "正在测试 DeepSeek API..." -ForegroundColor Cyan
Write-Host "  Base URL: https://api.deepseek.com"
Write-Host "  Model:    $Model"
Write-Host ""

try {
    $response = Invoke-RestMethod `
        -Uri "https://api.deepseek.com/chat/completions" `
        -Method Post `
        -Headers $headers `
        -Body $body `
        -TimeoutSec 30

    $reply = $response.choices[0].message.content
    Write-Host "API Key 验证成功!" -ForegroundColor Green
    Write-Host "模型回复: $reply"
    exit 0
}
catch {
    $status = $_.Exception.Response.StatusCode.value__
    Write-Host "API Key 验证失败 (HTTP $status)" -ForegroundColor Red
    Write-Host $_.Exception.Message
    if ($status -eq 401) {
        Write-Host "提示: Key 无效或格式错误，请到 https://platform.deepseek.com 检查" -ForegroundColor Yellow
    }
    elseif ($status -eq 402) {
        Write-Host "提示: 账户余额不足，请先充值" -ForegroundColor Yellow
    }
    exit 1
}
