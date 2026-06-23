# Katkı Yönergeleri (CONTRIBUTING)

Bu depo endüstri standartlarında bir Git iş akışı uygular. Lütfen aşağıdaki kurallara **harfiyen** uyun.

## 1. Dallanma Stratejisi (Branching Model)

- `main` ve `develop` dallarına **doğrudan push YASAKTIR.** Tüm geliştirmeler **Pull Request (PR)** ile yapılır.
- `main`: kararlı, yayınlanabilir sürüm. `develop`: aktif geliştirme dalı.
- Yeni dallar **`develop`** üzerinden açılır ve aşağıdaki standartla isimlendirilir:

| Amaç | Dal adı | Örnek |
|---|---|---|
| Yeni özellik | `feature/özellik-adı` | `feature/lane-following` |
| Hata düzeltme | `bugfix/hata-adı` | `bugfix/fix-yolo-conf` |
| Kritik canlı düzeltme | `hotfix/kritik-hata` | `hotfix/emergency-stop` |
| Sürüm | `release/vX.Y.Z` | `release/v1.0.0` |

## 2. Commit Mesajı Standardı (Conventional Commits)

Her commit şu şablona uymalıdır:

```
<tip>(<kapsam>): <açıklama>
```

| Tip | Kullanım | Örnek |
|---|---|---|
| `feat` | Yeni özellik | `feat(perception): add traffic light detection` |
| `fix` | Hata düzeltme | `fix(control): correct steering sign` |
| `docs` | Sadece dokümantasyon | `docs: update installation steps` |
| `style` | Biçim/boşluk/yazım | `style(planning): format with black` |
| `refactor` | Yeniden yapılandırma | `refactor(decision): simplify hfsm` |
| `test` | Test ekleme | `test(perception): add lane unit tests` |
| `chore` | Bağımlılık/yapılandırma | `chore: bump ultralytics` |

> Commit mesajları PR'da `commitlint` ile otomatik denetlenir; uymayan commit'ler reddedilir.

## 3. Pull Request (PR) Protokolü

1. `develop` üzerinden uygun isimli bir dal açın.
2. Değişikliklerinizi commit'leyip dalı `push` edin.
3. PR açın; **PR şablonunu eksiksiz doldurun** (ne yaptınız, hangi testler, hangi issue).
4. **CI kontrolleri (lint + test) geçmelidir.**
5. **En az 1 ekip üyesinin onayı (code review)** olmadan merge edilemez.
6. Merge sonrası dalınızı silin.

## 4. Otomasyon ve Kalite (CI/CD)

Her PR'da arka planda otomatik çalışır:
- **Linter** (`flake8`) — sözdizimi/ciddi hatalar merge'i engeller.
- **Testler** (`pytest tests/`).
- **Commitlint** — Conventional Commits denetimi.

Bu kontrollerden geçmeyen hiçbir kod hedef dala birleşemez.

## 5. Kod ve Dizin Düzeni

- Kaynak kod → `src/<modül>/`, testler → `tests/`, dokümanlar → `docs/`.
- Büyük model dosyaları (`*.pt`, `*.onnx`) depoya **eklenmez** (bkz. `.gitignore`); bağlantı/sürüm notu ile paylaşılır.
- Python kodu için satır uzunluğu ≤ 120; mümkünse `black` ile biçimlendirin.

Teşekkürler — temiz ve tutarlı bir depo hepimizin işini kolaylaştırır. 🚗💨
